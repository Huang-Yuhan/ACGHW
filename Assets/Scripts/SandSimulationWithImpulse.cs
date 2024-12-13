using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using Random = UnityEngine.Random;



public class SandSimulationWithImpulse : MonoBehaviour
{
    [Tooltip("实例化沙子的材质")]
    public Material material; 
    [Tooltip("实例化沙子的网格")]
    public Mesh mesh;
    [Tooltip("实例化沙子的数量")]
    public int sandCount = 256 * 256;
    public int granuleCount=>sandCount;
    public int particleCount => granuleCount * 4;
    [Tooltip("计算沙子的 ComputeShader")]
    public ComputeShader computeShader;
    [Tooltip("沙子的半径")]
    public float sandRadius = 0.1f;                         //同样也是粒子的半径
    [Tooltip("粒子的质量")] 
    public float particleMass = 1.0f;
    [Tooltip("弹性恢复系数")]
    public float restitutionCoefficient = 0.5f;
    [Tooltip("粒子的摩擦系数")]
    public float frictionCoefficient = 0.2f;
    [Tooltip("时间步长")]
    public float deltaTime = 0.02f;
    
    
    public bool isStopCompute = false;
    
    /// <summary>
    /// https://zh.wikipedia.org/wiki/%E6%AD%A3%E5%9B%9B%E9%9D%A2%E9%AB%94
    /// 以正四面体的中心作为原点建立三维直角坐标系的话，棱长a=2的正四面体的顶点坐标
    /// </summary>
    Vector3[] tetrahedronVertices = new Vector3[]
    {
        new Vector3(1,0,-1/(float)Math.Sqrt(2)),
        new Vector3(-1,0,-1/(float)Math.Sqrt(2)),
        new Vector3(0,1,1/(float)Math.Sqrt(2)),
        new Vector3(0,-1,1/(float)Math.Sqrt(2))
    };
    private int _kernel;
    private int _GridUpdatePreProcessKernel;
    private int _GridUpdateMainKernel;
    private int _GranuleDataSetIntoGridKernel;
    private ComputeBuffer _particlePositionBuffer;
    private ComputeBuffer _particleVelocityBuffer;
    private ComputeBuffer _granuleDataBuffer;
    private ComputeBuffer _gridCountBuffer;                                                  //记录每个格子中的粒子数量
    private ComputeBuffer _gridParticleBuffer;                                               //记录每个粒子所在的格子索引,每个grid中的粒子索引在该buffer中是连续的，通过_gridParticleBeginBuffer和_gridParticleEndBuffer可以找到每个格子中的粒子索引范围
    private ComputeBuffer _gridParticleBeginBuffer;                                          //记录每个格子中的第一个粒子在_gridParticleBuffer中的索引
    private ComputeBuffer _gridParticleCurrentBuffer;                                        //记录每个格子中的当前粒子在_gridParticleBuffer中的索引
    private ComputeBuffer _gridParticleEndBuffer;                                            //记录每个格子中的最后一个粒子在_gridParticleBuffer中的索引
    private ComputeBuffer _columnSumBuffer;
    private ComputeBuffer _prefixSumBuffer;
    private RenderParams _renderParams;
    private GraphicsBuffer _commandBuffer;
    private GraphicsBuffer.IndirectDrawIndexedArgs[] _commandData;
    private int _bufferIndexBegin;                     //取值为0或1，用于交替读写两个缓冲区
    private Matrix4x4 _granuleInertiaReferenceTensor;
    
    //ids
    private int _particlePositionBufferId;
    private int _particleVelocityBufferId;
    private int _granuleDataBufferId;
    private int _granuleCountId;
    private int _particleCountId;
    private int _particleMassId;
    private int _deltaTimeId;
    private int _particleRadiusId;
    private int _bufferIndexBeginId;
    private int _granuleInertiaReferenceTensorId;
    private int _RigidBodyParticleBufferId;
    private int _rigidBodyParticleCountId;
    private int _granuleDebugDataBufferId;
    private int _restitutionCoefficientId;
    private int _frictionCoefficientId;
    private int _planeDataBufferId;
    private int _planeCountId;
    private int _gridCountBufferId;
    private int _gridGranuleBufferId;
    private int _gridGranuleBeginBufferId;
    private int _gridGranuleCurrentBufferId;
    private int _gridGranuleEndBufferId;
    private int _gridResolutionId;
    private int _columnSumBufferId;
    private int _prefixSumBufferId;
    private int _gridCellSizeId;
    
    //grid
    private float gridCellSize;
    private Vector3Int gridResolution;

    private int[] gridCount;
    private int[] gridParticle;
    private int[] gridParticleBegin;
    private int[] gridParticleCurrent;
    private int[] gridParticleEnd;
    private int[] columnSum;
    private int[] prefixSum;
    
    
    struct GranuleDataType
    {
        public Vector3 Position;
        public Vector3 Velocity;
        public Vector3 AngularVelocity;
        public Quaternion Rotation;
        public static int GetSize()
        {
            return sizeof(float) * 3 * 3 + sizeof(float) * 4;
        }
    }

    struct GranuleDebugDataType
    {
        public Vector3 Position;
        public Vector3 Velocity;
        public Vector3 AngularVelocity;
        public Quaternion Rotation;
        public Vector3 Force;
        public Vector3 Torque;
        public Vector3 Acceleration;
        public static int GetSize()
        {
            return sizeof(float) * 3 * 6 + sizeof(float) * 4;
        }
    }
    private ComputeBuffer _granuleDebugDataBuffer;
    
    Vector3Int GetGridCellIndex(Vector3 position)
    {
        //这里限定我们只在[-5,5]*[0,+inf]*[-5,5]的区域内进行模拟
        
        Vector3 offset = position- new Vector3(-5,0,-5);
        return new Vector3Int(Mathf.FloorToInt(offset.x / gridCellSize), Mathf.FloorToInt(offset.y / gridCellSize), Mathf.FloorToInt(offset.z / gridCellSize));
    }
    
    int GetGridCellInex1D(Vector3Int index)
    {
        return index.x + index.y * gridResolution.x + index.z * gridResolution.x * gridResolution.y;
    }
    
    void SetupCommandBuffer()
    {
        _commandBuffer = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments, 1, GraphicsBuffer.IndirectDrawIndexedArgs.size);
        _commandData = new GraphicsBuffer.IndirectDrawIndexedArgs[1];
        _commandData[0].indexCountPerInstance = mesh.GetIndexCount(0);
        _commandData[0].instanceCount = (uint)particleCount;
        _commandBuffer.SetData(_commandData);
        //设置Scale
        material.SetFloat("_Radius", sandRadius);
        _renderParams = new RenderParams(material);
        _renderParams.worldBounds = new Bounds(Vector3.zero, 100 * Vector3.one);//设定边界
        
        //设置ids
        _particlePositionBufferId = Shader.PropertyToID("_ParticlePositionBuffer");
        _particleVelocityBufferId = Shader.PropertyToID("_ParticleVelocityBuffer");
        _granuleDataBufferId = Shader.PropertyToID("_GranuleBuffer");
        _granuleCountId = Shader.PropertyToID("_GranuleCount");
        _particleMassId = Shader.PropertyToID("_ParticleMass");
        _deltaTimeId = Shader.PropertyToID("_DeltaTime");
        _particleRadiusId = Shader.PropertyToID("_ParticleRadius");
        _particleCountId = Shader.PropertyToID("_ParticleCount");
        _bufferIndexBeginId = Shader.PropertyToID("_bufferIndexBegin");
        _granuleInertiaReferenceTensorId = Shader.PropertyToID("_GranuleInertiaReferenceTensor");
        _RigidBodyParticleBufferId = Shader.PropertyToID("_RigidBodyParticleBuffer");
        _rigidBodyParticleCountId = Shader.PropertyToID("_RigidBodyParticleCount");
        _granuleDebugDataBufferId = Shader.PropertyToID("_GranuleDebugBuffer");
        _restitutionCoefficientId = Shader.PropertyToID("_RestitutionCoefficient");
        _frictionCoefficientId = Shader.PropertyToID("_FrictionCoefficient");
        _planeDataBufferId = Shader.PropertyToID("_PlaneBuffer");
        _planeCountId = Shader.PropertyToID("_PlaneCount");
        _gridCountBufferId = Shader.PropertyToID("_GridCountBuffer");
        _gridGranuleBufferId = Shader.PropertyToID("_GridGranuleBuffer");
        _gridGranuleBeginBufferId = Shader.PropertyToID("_GridGranuleBeginBuffer");
        _gridGranuleCurrentBufferId = Shader.PropertyToID("_GridGranuleCurrentBuffer");
        _gridGranuleEndBufferId = Shader.PropertyToID("_GridGranuleEndBuffer");
        _gridResolutionId = Shader.PropertyToID("_GridResolution");
        _columnSumBufferId = Shader.PropertyToID("_ColumnSumBuffer");
        _prefixSumBufferId = Shader.PropertyToID("_PrefixSumBuffer");
        _gridCellSizeId = Shader.PropertyToID("_GridCellSize");
    }
    
    void SetupSimulation()
    {
        _kernel = computeShader.FindKernel("CSMain");
        _GridUpdatePreProcessKernel = computeShader.FindKernel("GridUpdatePreProcess");
        _GridUpdateMainKernel = computeShader.FindKernel("GridUpdateMain");
        _GranuleDataSetIntoGridKernel = computeShader.FindKernel("GranuleDataSetIntoGrid");
        Vector3[] particlePositions = new Vector3[particleCount*2];
        Vector3[] particleVelocities = new Vector3[particleCount*2];
        GranuleDataType[] granuleData = new GranuleDataType[granuleCount*2];
        
        //grid
        
                
        //初始化网格
        gridCellSize = 5 * sandRadius;
        //这里限定我们只在[-5,5]*[0,+inf]*[-5,5]的区域内进行模拟
        gridResolution = new Vector3Int(Mathf.CeilToInt(10 / gridCellSize), Mathf.CeilToInt(10 / gridCellSize), Mathf.CeilToInt(10 / gridCellSize));
        //初始化grid相关数据
        gridCount = new int[gridResolution.x * gridResolution.y * gridResolution.z];
        gridParticle = new int[particleCount];
        gridParticleBegin = new int[gridResolution.x * gridResolution.y * gridResolution.z];
        gridParticleCurrent = new int[gridResolution.x * gridResolution.y * gridResolution.z];
        gridParticleEnd = new int[gridResolution.x * gridResolution.y * gridResolution.z];
        columnSum = new int[gridResolution.y * gridResolution.z];
        prefixSum = new int[gridResolution.y * gridResolution.z];

        
        _bufferIndexBegin = 0;
        for(int i = 0; i < granuleCount; i++)
        {
            granuleData[i].Position = new Vector3(Random.Range(-4.0f,4.0f), Random.Range(2.0f, 5.0f), Random.Range(-4.0f,4.0f));
            granuleData[i].Velocity = Random.onUnitSphere * 10;
            granuleData[i].AngularVelocity = Vector3.zero;
            granuleData[i].Rotation = Quaternion.identity;
            granuleData[i+granuleCount]=granuleData[i];
            for(int j = 0; j < 4; j++)
            {
                particlePositions[i * 4 + j] = granuleData[i].Position + tetrahedronVertices[j] * sandRadius / 2;

            }
            //更新grid中的信息
            Vector3Int gridIndex = GetGridCellIndex(granuleData[i].Position);
            gridCount[GetGridCellInex1D(gridIndex)]++;
        }
        
        //初始化grid相关buffer
        _gridCountBuffer = new ComputeBuffer(gridCount.Length, sizeof(int));
        _gridParticleBuffer = new ComputeBuffer(particleCount, sizeof(int));
        _gridParticleBeginBuffer = new ComputeBuffer(gridParticleBegin.Length, sizeof(int));
        _gridParticleCurrentBuffer = new ComputeBuffer(gridParticleBegin.Length, sizeof(int));
        _gridParticleEndBuffer = new ComputeBuffer(gridParticleEnd.Length, sizeof(int));
        _columnSumBuffer = new ComputeBuffer(columnSum.Length, sizeof(int));
        _prefixSumBuffer = new ComputeBuffer(prefixSum.Length, sizeof(int));
        

        
        
        
        
        /*// //TEST
        gridCount[GetGridCellInex1D(GetGridCellIndex(granuleData[0].Position))] = 0;
        gridCount[GetGridCellInex1D(GetGridCellIndex(granuleData[1].Position))] = 0;
        granuleData[0].Position = new Vector3(0.5f, 5, 0.5f);
        granuleData[0].Velocity = Vector3.zero;
        granuleData[1].Position = new Vector3(0.5f, 3,0.5f);
        granuleData[1].Velocity = Vector3.zero;
        gridCount[GetGridCellInex1D(GetGridCellIndex(granuleData[0].Position))]++;
        gridCount[GetGridCellInex1D(GetGridCellIndex(granuleData[1].Position))]++;*/
        
        _particlePositionBuffer = new ComputeBuffer(particleCount*2, sizeof(float) * 3);
        _particleVelocityBuffer = new ComputeBuffer(particleCount*2, sizeof(float) * 3);
        _granuleDataBuffer = new ComputeBuffer(granuleCount*2, GranuleDataType.GetSize());
        
        _particlePositionBuffer.SetData(particlePositions);
        _particleVelocityBuffer.SetData(particleVelocities);
        _granuleDataBuffer.SetData(granuleData);
        

        _granuleInertiaReferenceTensor = CalculateRefererenceInertiaTensor();
        

        _granuleDebugDataBuffer = new ComputeBuffer(granuleCount, GranuleDebugDataType.GetSize());
        
                
        _gridCountBuffer.SetData(gridCount);
        _gridParticleBuffer.SetData(gridParticle);
        _gridParticleBeginBuffer.SetData(gridParticleBegin);
        _gridParticleEndBuffer.SetData(gridParticleEnd);
        

    }
    
    void Setup()
    {
        SetupCommandBuffer();
        SetupSimulation();
    }

    private void OnDestroy()
    {
        _particlePositionBuffer?.Release();
        _particleVelocityBuffer?.Release();
        _granuleDataBuffer?.Release();
        _commandBuffer?.Release();
        _granuleDebugDataBuffer?.Release();
        _gridCountBuffer?.Release();
        _gridParticleBuffer?.Release();
        _gridParticleBeginBuffer?.Release();
        _gridParticleCurrentBuffer?.Release();
        _gridParticleEndBuffer?.Release();
        _columnSumBuffer?.Release();
        _prefixSumBuffer?.Release();
    }

    private void Start()
    {
        Setup();
    }

    void GridUpdate()
    {
        computeShader.SetVector(_gridResolutionId, new Vector4(gridResolution.x, gridResolution.y, gridResolution.z, 0));
        computeShader.SetBuffer(_GridUpdatePreProcessKernel, _gridCountBufferId, _gridCountBuffer);
        computeShader.SetBuffer(_GridUpdatePreProcessKernel, _columnSumBufferId, _columnSumBuffer);
        computeShader.SetBuffer(_GridUpdatePreProcessKernel, _prefixSumBufferId, _prefixSumBuffer);
        computeShader.Dispatch(_GridUpdatePreProcessKernel, gridResolution.y/8, gridResolution.z/8, 1);
        
        _columnSumBuffer.GetData(columnSum);
        for(int i=0;i<gridResolution.y * gridResolution.z;i++)
        {
            prefixSum[i] = 0;
            if(i > 0) prefixSum[i] += prefixSum[i - 1]+columnSum[i - 1];
        }
        _prefixSumBuffer.SetData(prefixSum);
        
        
        computeShader.SetBuffer(_GridUpdateMainKernel, _gridCountBufferId, _gridCountBuffer);
        computeShader.SetBuffer(_GridUpdateMainKernel, _prefixSumBufferId, _prefixSumBuffer);
        computeShader.SetBuffer(_GridUpdateMainKernel,_gridGranuleBeginBufferId, _gridParticleBeginBuffer);
        computeShader.SetBuffer(_GridUpdateMainKernel,_gridGranuleCurrentBufferId, _gridParticleCurrentBuffer);
        computeShader.SetBuffer(_GridUpdateMainKernel,_gridGranuleEndBufferId, _gridParticleEndBuffer);
        
        computeShader.Dispatch(_GridUpdateMainKernel, gridResolution.y/8, gridResolution.z/8, 1);
        
        //GranuleDataSetIntoGrid
        computeShader.SetBuffer(_GranuleDataSetIntoGridKernel, _granuleDataBufferId, _granuleDataBuffer);
        computeShader.SetBuffer(_GranuleDataSetIntoGridKernel,_gridGranuleBeginBufferId, _gridParticleBeginBuffer);
        computeShader.SetBuffer(_GranuleDataSetIntoGridKernel,_gridGranuleEndBufferId, _gridParticleEndBuffer);
        computeShader.SetBuffer(_GranuleDataSetIntoGridKernel,_gridGranuleBufferId, _gridParticleBuffer);
        computeShader.SetBuffer(_GranuleDataSetIntoGridKernel,_gridGranuleCurrentBufferId, _gridParticleCurrentBuffer);
        computeShader.Dispatch(_GranuleDataSetIntoGridKernel, Mathf.Max(1,granuleCount/32), 1, 1);
        
        
        
    }

    private void FixedUpdate()
    {
        if(isStopCompute) return;
        
        computeShader.SetBuffer(_kernel, _particlePositionBufferId, _particlePositionBuffer);
        computeShader.SetBuffer(_kernel, _particleVelocityBufferId, _particleVelocityBuffer);
        computeShader.SetBuffer(_kernel, _granuleDataBufferId, _granuleDataBuffer);
        if(ParticelRegisterManager.Instance.rigidBodyParticleDatas.Count > 0) computeShader.SetBuffer(_kernel, _RigidBodyParticleBufferId, ParticelRegisterManager.Instance._particleBuffer);
        computeShader.SetBuffer(_kernel, _granuleDebugDataBufferId, _granuleDebugDataBuffer);
        if(PlaneRegisterManager.Instance.planeDatas.Count > 0) computeShader.SetBuffer(_kernel, _planeDataBufferId, PlaneRegisterManager.Instance._planeBuffer);
        computeShader.SetBuffer(_kernel, _gridCountBufferId, _gridCountBuffer);
        computeShader.SetBuffer(_kernel, _gridGranuleBufferId, _gridParticleBuffer);
        computeShader.SetBuffer(_kernel, _gridGranuleBeginBufferId, _gridParticleBeginBuffer);
        computeShader.SetBuffer(_kernel, _gridGranuleCurrentBufferId, _gridParticleBuffer);
        computeShader.SetBuffer(_kernel, _gridGranuleEndBufferId, _gridParticleEndBuffer);
        
        computeShader.SetInt(_granuleCountId, granuleCount);
        computeShader.SetFloat(_particleMassId, particleMass);
        computeShader.SetFloat(_deltaTimeId, deltaTime);
        computeShader.SetFloat(_particleRadiusId, sandRadius);  
        computeShader.SetInt(_particleCountId, particleCount);
        computeShader.SetInt(_bufferIndexBeginId, _bufferIndexBegin);
        computeShader.SetMatrix(_granuleInertiaReferenceTensorId, _granuleInertiaReferenceTensor);
        computeShader.SetInt(_rigidBodyParticleCountId, ParticelRegisterManager.Instance.rigidBodyParticleDatas.Count);
        computeShader.SetFloat(_restitutionCoefficientId, restitutionCoefficient);
        computeShader.SetFloat(_frictionCoefficientId, frictionCoefficient);
        computeShader.SetInt(_planeCountId, PlaneRegisterManager.Instance.planeDatas.Count);
        computeShader.SetVector(_gridResolutionId, new Vector4(gridResolution.x, gridResolution.y, gridResolution.z, 0));
        computeShader.SetFloat(_gridCellSizeId, gridCellSize);
        
        
        computeShader.Dispatch(_kernel, Math.Max(1, granuleCount / 32), 1, 1);                          //进行物理模拟
        
        GridUpdate();
        
        _bufferIndexBegin = 1 - _bufferIndexBegin;
        
        //DebugGranuleBuffer();
    }

    private void Update()
    {
        material.SetBuffer("_ParticlePositionBuffer", _particlePositionBuffer);//只有_ParticlePositionBuffer需要传递给材质
        material.SetInt("_BufferBeginIndex", _bufferIndexBegin*particleCount);
        Graphics.RenderMeshIndirect(_renderParams, mesh, _commandBuffer, 1);                                                //渲染
    }

    void DebugParticleBuffer()
    {
        Vector3[] particlePositions = new Vector3[particleCount];
        Vector3[] particleVelocities = new Vector3[particleCount];
        _particlePositionBuffer.GetData(particlePositions);
        _particleVelocityBuffer.GetData(particleVelocities);
        for (int i = 0; i < particleCount; i++)
        {
            Debug.LogFormat("particlePositions[{0}]: {1}, particleVelocities[{0}]: {2}", i, particlePositions[i],
                particleVelocities[i]);
        }
    }
    
    void DebugGranuleBuffer()
    {
        GranuleDebugDataType[] granuleDebugData = new GranuleDebugDataType[granuleCount];
        _granuleDebugDataBuffer.GetData(granuleDebugData);
        for (int i = 0; i < granuleCount; i++)
        {
            Debug.LogFormat("granuleDebugData[{0}].position:{1}", i, granuleDebugData[i].Position);
        }
        
    }
    
    /// <summary>
    /// 由于四面体的对称性，可以直接计算出参考惯性张量，发现这是一个对角矩阵，因此可以简化计算他的逆矩阵
    /// </summary>
    /// <returns></returns>
    Matrix4x4 CalculateRefererenceInertiaTensor()
    {
        Mesh sphereMesh = mesh;
        Vector3[] vertices = sphereMesh.vertices;
        Matrix4x4 I_ref = Matrix4x4.zero;
        float vertexMass = particleMass;
        for (int i = 0; i < tetrahedronVertices.Length; i++)
        {
            I_ref[0, 0] += vertexMass * tetrahedronVertices[i].sqrMagnitude;
            I_ref[1, 1] += vertexMass * tetrahedronVertices[i].sqrMagnitude;
            I_ref[2, 2] += vertexMass * tetrahedronVertices[i].sqrMagnitude;
            I_ref[0, 0] -= vertexMass * tetrahedronVertices[i][0] * tetrahedronVertices[i][0];
            I_ref[0, 1] -= vertexMass * tetrahedronVertices[i][0] * tetrahedronVertices[i][1];
            I_ref[0, 2] -= vertexMass * tetrahedronVertices[i][0] * tetrahedronVertices[i][2];
            I_ref[1, 0] -= vertexMass * tetrahedronVertices[i][1] * tetrahedronVertices[i][0];
            I_ref[1, 1] -= vertexMass * tetrahedronVertices[i][1] * tetrahedronVertices[i][1];
            I_ref[1, 2] -= vertexMass * tetrahedronVertices[i][1] * tetrahedronVertices[i][2];
            I_ref[2, 0] -= vertexMass * tetrahedronVertices[i][2] * tetrahedronVertices[i][0];
            I_ref[2, 1] -= vertexMass * tetrahedronVertices[i][2] * tetrahedronVertices[i][1];
            I_ref[2, 2] -= vertexMass * tetrahedronVertices[i][2] * tetrahedronVertices[i][2];
        }
        I_ref[3, 3] = 1;                
        Debug.Log("I_ref: " + I_ref);
        return I_ref;
    }
}
