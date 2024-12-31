using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using Random = UnityEngine.Random;



public class ThreeDSandPainting : MonoBehaviour
{
    [Tooltip("实例化沙子的材质")]
    public Material material; 
    [Tooltip("实例化沙子的网格")]
    public Mesh mesh;
    [Tooltip("实例化沙子的最大数量")]
    public int maxSandCount = 256 * 256;
    public int maxGranuleCount=>maxSandCount;
    public int maxParticleCount => maxSandCount * 4;
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
    
    private int currentSandCount = 0;
    private int currentParticleCount => currentSandCount * 4;
    private int currentGranuleCount => currentSandCount;
    
    private int _kernel;
    private int _GridUpdatePreProcessKernel;
    private int _GridUpdateMainKernel;
    private int _GranuleDataSetIntoGridKernel;
    private int _ComsumeGranuleKernel;
    private ComputeBuffer _particlePositionBuffer;
    private ComputeBuffer _particleVelocityBuffer;
    private ComputeBuffer _particleColorBuffer;
    private ComputeBuffer _granuleDataBuffer;
    private ComputeBuffer _gridCountBuffer;                                                  //记录每个格子中的粒子数量
    private ComputeBuffer _gridParticleBuffer;                                               //记录每个粒子所在的格子索引,每个grid中的粒子索引在该buffer中是连续的，通过_gridParticleBeginBuffer和_gridParticleEndBuffer可以找到每个格子中的粒子索引范围
    private ComputeBuffer _gridParticleBeginBuffer;                                          //记录每个格子中的第一个粒子在_gridParticleBuffer中的索引
    private ComputeBuffer _gridParticleCurrentBuffer;                                        //记录每个格子中的当前粒子在_gridParticleBuffer中的索引
    private ComputeBuffer _gridParticleEndBuffer;                                            //记录每个格子中的最后一个粒子在_gridParticleBuffer中的索引
    private ComputeBuffer _columnSumBuffer;
    private ComputeBuffer _prefixSumBuffer;
    private ComputeBuffer _comsumeGranuleBuffer;                                             //用于动态添加沙粒                     
    private RenderParams _renderParams;
    private GraphicsBuffer _commandBuffer;
    private GraphicsBuffer.IndirectDrawIndexedArgs[] _commandData;
    private int _bufferIndexBegin;                     //取值为0或1，用于交替读写两个缓冲区
    private Matrix4x4 _granuleInertiaReferenceTensor;
    
    //ids
    private int _particlePositionBufferId;
    private int _particleVelocityBufferId;
    private int _particleColorBufferId;
    private int _granuleDataBufferId;
    private int _granuleCountId;
    private int _maxGranuleCountId;
    private int _particleCountId;
    private int _maxParticleCountId;
    private int _particleMassId;
    private int _deltaTimeId;
    private int _particleRadiusId;
    private int _bufferIndexBeginId;
    private int _granuleInertiaReferenceTensorId;
    private int _RigidBodyParticleBufferId;
    private int _rigidBodyParticleCountId;
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
    private int _ConsumeGranuleCountId;
    
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
    
    //dynamic add sand
    
    
    float lastTime = 0;
    List<GranuleDataType> granuleDataList = new List<GranuleDataType>();
    
    struct GranuleDataType
    {
        public Vector3 Position;
        public Vector3 Velocity;
        public Vector3 AngularVelocity;
        public Quaternion Rotation;
        public Vector3 Color;
        public static int GetSize()
        {
            return sizeof(float) * 3 * 4 + sizeof(float) * 4;
        }
    }
    
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
        _commandData[0].instanceCount = (uint)currentGranuleCount;
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
        _maxGranuleCountId = Shader.PropertyToID("_MaxGranuleCount");
        _particleMassId = Shader.PropertyToID("_ParticleMass");
        _deltaTimeId = Shader.PropertyToID("_DeltaTime");
        _particleRadiusId = Shader.PropertyToID("_ParticleRadius");
        _particleCountId = Shader.PropertyToID("_ParticleCount");
        _maxParticleCountId = Shader.PropertyToID("_MaxParticleCount");
        _bufferIndexBeginId = Shader.PropertyToID("_bufferIndexBegin");
        _granuleInertiaReferenceTensorId = Shader.PropertyToID("_GranuleInertiaReferenceTensor");
        _RigidBodyParticleBufferId = Shader.PropertyToID("_RigidBodyParticleBuffer");
        _rigidBodyParticleCountId = Shader.PropertyToID("_RigidBodyParticleCount");
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
        _ConsumeGranuleCountId = Shader.PropertyToID("_ConsumeGranuleCount");
        _particleColorBufferId = Shader.PropertyToID("_ParticleColorBuffer");
        
        //mesh = GetSphereLowPolyMesh();
        
    }
    
    void SetupSimulation()
    {
        _kernel = computeShader.FindKernel("CSMain");
        _GridUpdatePreProcessKernel = computeShader.FindKernel("GridUpdatePreProcess");
        _GridUpdateMainKernel = computeShader.FindKernel("GridUpdateMain");
        _GranuleDataSetIntoGridKernel = computeShader.FindKernel("GranuleDataSetIntoGrid");
        _ComsumeGranuleKernel = computeShader.FindKernel("DynamicAddSand");
        Vector3[] particlePositions = new Vector3[maxParticleCount * 2];
        Vector3[] particleVelocities = new Vector3[maxParticleCount*2];
        GranuleDataType[] granuleData = new GranuleDataType[maxGranuleCount*2];
        
        //grid
        
                
        //初始化网格
        gridCellSize = 5 * sandRadius;
        //这里限定我们只在[-5,5]*[0,+inf]*[-5,5]的区域内进行模拟
        gridResolution = new Vector3Int(Mathf.CeilToInt(10 / gridCellSize), Mathf.CeilToInt(10 / gridCellSize), Mathf.CeilToInt(10 / gridCellSize));
        //初始化grid相关数据
        gridCount = new int[gridResolution.x * gridResolution.y * gridResolution.z];
        gridParticle = new int[maxParticleCount];
        gridParticleBegin = new int[gridResolution.x * gridResolution.y * gridResolution.z];
        gridParticleCurrent = new int[gridResolution.x * gridResolution.y * gridResolution.z];
        gridParticleEnd = new int[gridResolution.x * gridResolution.y * gridResolution.z];
        columnSum = new int[gridResolution.y * gridResolution.z];
        prefixSum = new int[gridResolution.y * gridResolution.z];

        
        _bufferIndexBegin = 0;
  
        
        //初始化grid相关buffer
        _gridCountBuffer = new ComputeBuffer(gridCount.Length, sizeof(int));
        _gridParticleBuffer = new ComputeBuffer(maxParticleCount, sizeof(int));
        _gridParticleBeginBuffer = new ComputeBuffer(gridParticleBegin.Length, sizeof(int));
        _gridParticleCurrentBuffer = new ComputeBuffer(gridParticleBegin.Length, sizeof(int));
        _gridParticleEndBuffer = new ComputeBuffer(gridParticleEnd.Length, sizeof(int));
        _columnSumBuffer = new ComputeBuffer(columnSum.Length, sizeof(int));
        _prefixSumBuffer = new ComputeBuffer(prefixSum.Length, sizeof(int));
        
        _particlePositionBuffer = new ComputeBuffer(maxParticleCount*2, sizeof(float) * 3);
        _particleVelocityBuffer = new ComputeBuffer(maxParticleCount*2, sizeof(float) * 3);
        _particleColorBuffer = new ComputeBuffer(maxParticleCount, sizeof(float) * 3);
        _granuleDataBuffer = new ComputeBuffer(maxGranuleCount*2, GranuleDataType.GetSize());
        _comsumeGranuleBuffer = new ComputeBuffer(maxGranuleCount, GranuleDataType.GetSize(), ComputeBufferType.Append);
        
        
        _particlePositionBuffer.SetData(particlePositions);
        _particleVelocityBuffer.SetData(particleVelocities);
        _granuleDataBuffer.SetData(granuleData);
        

        _granuleInertiaReferenceTensor = CalculateRefererenceInertiaTensor();
        

        
                
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
        _gridCountBuffer?.Release();
        _gridParticleBuffer?.Release();
        _gridParticleBeginBuffer?.Release();
        _gridParticleCurrentBuffer?.Release();
        _gridParticleEndBuffer?.Release();
        _columnSumBuffer?.Release();
        _prefixSumBuffer?.Release();
        _comsumeGranuleBuffer?.Release();
        _particleColorBuffer?.Release();
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
        computeShader.Dispatch(_GranuleDataSetIntoGridKernel, Mathf.CeilToInt(currentGranuleCount/32f), 1, 1);
        
        
        
    }

    private Vector3 randomColor;
    private void FixedUpdate()
    {
        
        //进行动态添加沙粒
        if(Input.GetMouseButton(0) && Time.time - lastTime > 1f)
        {
            lastTime = Time.time;
            randomColor = new Vector3(Random.Range(0, 1f), Random.Range(0, 1f), Random.Range(0, 1f));
            AddSandStep(randomColor);
        }else if(Input.GetMouseButton(1) && Time.time - lastTime > 1f)
        {
            lastTime = Time.time;
            AddSandStep(randomColor);
        }
        
        
        if (currentSandCount == 0) return;

        
        computeShader.SetBuffer(_kernel, _particlePositionBufferId, _particlePositionBuffer);
        computeShader.SetBuffer(_kernel, _particleVelocityBufferId, _particleVelocityBuffer);
        computeShader.SetBuffer(_kernel, _granuleDataBufferId, _granuleDataBuffer);
        if(ParticelRegisterManager.Instance.rigidBodyParticleDatas.Count > 0) computeShader.SetBuffer(_kernel, _RigidBodyParticleBufferId, ParticelRegisterManager.Instance._particleBuffer);

        if(PlaneRegisterManager.Instance.planeDatas.Count > 0) computeShader.SetBuffer(_kernel, _planeDataBufferId, PlaneRegisterManager.Instance._planeBuffer);
        computeShader.SetBuffer(_kernel, _gridCountBufferId, _gridCountBuffer);
        computeShader.SetBuffer(_kernel, _gridGranuleBufferId, _gridParticleBuffer);
        computeShader.SetBuffer(_kernel, _gridGranuleBeginBufferId, _gridParticleBeginBuffer);
        computeShader.SetBuffer(_kernel, _gridGranuleCurrentBufferId, _gridParticleBuffer);
        computeShader.SetBuffer(_kernel, _gridGranuleEndBufferId, _gridParticleEndBuffer);
        
        computeShader.SetInt(_granuleCountId, currentGranuleCount);
        computeShader.SetInt(_maxGranuleCountId, maxGranuleCount);
        computeShader.SetFloat(_particleMassId, particleMass);
        computeShader.SetFloat(_deltaTimeId, deltaTime);
        computeShader.SetFloat(_particleRadiusId, sandRadius);  
        computeShader.SetInt(_particleCountId, currentParticleCount);
        computeShader.SetInt(_maxParticleCountId, maxParticleCount);
        computeShader.SetInt(_bufferIndexBeginId, _bufferIndexBegin);
        computeShader.SetMatrix(_granuleInertiaReferenceTensorId, _granuleInertiaReferenceTensor);
        computeShader.SetInt(_rigidBodyParticleCountId, ParticelRegisterManager.Instance.rigidBodyParticleDatas.Count);
        computeShader.SetFloat(_restitutionCoefficientId, restitutionCoefficient);
        computeShader.SetFloat(_frictionCoefficientId, frictionCoefficient);
        computeShader.SetInt(_planeCountId, PlaneRegisterManager.Instance.planeDatas.Count);
        computeShader.SetVector(_gridResolutionId, new Vector4(gridResolution.x, gridResolution.y, gridResolution.z, 0));
        computeShader.SetFloat(_gridCellSizeId, gridCellSize);
        
        
        computeShader.Dispatch(_kernel,Mathf.CeilToInt(currentGranuleCount/32f) , 1, 1);                          //进行物理模拟
        
        GridUpdate();
        
        _bufferIndexBegin = 1 - _bufferIndexBegin;
        
    }

    private void Update()
    {
        material.SetBuffer("_ParticlePositionBuffer", _particlePositionBuffer);//只有_ParticlePositionBuffer需要传递给材质
        material.SetBuffer("_ParticleColorBuffer", _particleColorBuffer);
        material.SetInt("_BufferBeginIndex", _bufferIndexBegin*maxParticleCount);
        _commandData[0].instanceCount = (uint)currentParticleCount;
        _commandBuffer.SetData(_commandData);
        Graphics.RenderMeshIndirect(_renderParams, mesh, _commandBuffer, 1);                                                //渲染
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

    Mesh GetSphereLowPolyMesh()
    {
        int n = 10;
        float r = 0.5f;
        
        Mesh mesh = new Mesh();
        //球的一份的弧度
        float ang = 2 * Mathf.PI / n;
        //存放球的各个顶点坐标
        List<Vector3> vers = new List<Vector3>();
        //存放读取的顶点坐标的顺序
        List<int> tris = new List<int>();
        for (int i = 0; i < n/2+1; i++)
        {
            float xr = Mathf.Sin(i * ang) * r;
            float y = Mathf.Cos(i * ang) * r;
            for (int j = 0; j < n; j++)
            {
                float x = Mathf.Sin(j * ang) * xr;
                float z = Mathf.Cos(j * ang) * xr;
                vers.Add(new Vector3(x, y, z));
                if (j==n-1)
                {
                    float x0 = Mathf.Sin(0) * xr;
                    float z0 = Mathf.Cos(0) * xr;
                    vers.Add(new Vector3(x0, y, z0));
                }
                if (i<n/2&&j<n)
                {
                    tris.Add(i * (n + 1) + j);
                    tris.Add(i * (n + 1) + j + 1);
                    tris.Add((i + 1) * (n + 1) + j);
                    tris.Add((i + 1) * (n + 1) + j);
                    tris.Add(i * (n + 1) + j + 1);
                    tris.Add((i + 1) * (n + 1) + j + 1);
                }
            }
        }
        //赋值
        mesh.vertices = vers.ToArray();
        mesh.triangles = tris.ToArray();
        mesh.RecalculateNormals();
        
       return mesh;
    }
    

    
    void AddSandStep(Vector3 color)
    {
        //在[-1.5,1.5]*[0,+inf]*[-1.5,1.5]的区域内随机生成沙粒
        for (int i = 0; i < 1024; i++)
        {
            var pos = new Vector3(Random.Range(-1.6f, 1.6f), 5f, Random.Range(-1.6f, 1.6f));
            AddSand(pos, Vector3.zero, Vector3.zero, Quaternion.identity, color);
        }
        AddSandInCS();
    }

    void AddSandInCS()
    {
        if(granuleDataList.Count == 0) return;
        _comsumeGranuleBuffer.SetCounterValue(0);
        _comsumeGranuleBuffer.SetData(granuleDataList);
        _comsumeGranuleBuffer.SetCounterValue((uint)granuleDataList.Count);
        computeShader.SetInt(_ConsumeGranuleCountId, granuleDataList.Count);
        computeShader.SetBuffer(_ComsumeGranuleKernel, "_readyToConsumeGranuleBuffer", _comsumeGranuleBuffer);
        computeShader.SetBuffer(_ComsumeGranuleKernel, _granuleDataBufferId, _granuleDataBuffer);
        computeShader.SetInt(_granuleCountId, currentGranuleCount);
        computeShader.SetInt(_maxGranuleCountId, maxGranuleCount);
        computeShader.SetFloat(_particleRadiusId, sandRadius);
        computeShader.SetBuffer(_ComsumeGranuleKernel,_particlePositionBufferId, _particlePositionBuffer);
        computeShader.SetBuffer(_ComsumeGranuleKernel,_particleVelocityBufferId, _particleVelocityBuffer);
        computeShader.SetBuffer(_ComsumeGranuleKernel,_gridCountBufferId, _gridCountBuffer);
        computeShader.SetVector(_gridResolutionId, new Vector4(gridResolution.x, gridResolution.y, gridResolution.z, 0));
        computeShader.SetFloat(_gridCellSizeId, gridCellSize);
        computeShader.SetBuffer(_ComsumeGranuleKernel,_particleColorBufferId, _particleColorBuffer);
        computeShader.Dispatch(_ComsumeGranuleKernel, Mathf.CeilToInt(granuleDataList.Count/32f), 1, 1);
        
        currentSandCount += granuleDataList.Count;
        granuleDataList.Clear();
    }
    
    void AddSand(Vector3 position, Vector3 velocity, Vector3 angularVelocity, Quaternion rotation, Vector3 color)
    {
        if(currentSandCount >= maxSandCount) return;
        granuleDataList.Add(new GranuleDataType()
        {
            Position = position,
            Velocity = velocity,
            AngularVelocity = angularVelocity,
            Rotation = rotation,
            Color = color
        });

    }
    
}
