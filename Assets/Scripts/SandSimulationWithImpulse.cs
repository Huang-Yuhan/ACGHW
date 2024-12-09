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
    private ComputeBuffer _particlePositionBuffer;
    private ComputeBuffer _particleVelocityBuffer;
    private ComputeBuffer _granuleDataBuffer;
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
    }
    
    void SetupSimulation()
    {
        _kernel = computeShader.FindKernel("CSMain");
        Vector3[] particlePositions = new Vector3[particleCount*2];
        Vector3[] particleVelocities = new Vector3[particleCount*2];
        GranuleDataType[] granuleData = new GranuleDataType[granuleCount*2];
        _bufferIndexBegin = 0;
        for(int i = 0; i < granuleCount; i++)
        {
            granuleData[i].Position = new Vector3(Random.Range(-3.0f,3.0f), Random.Range(3.0f, 5.0f), Random.Range(-3.0f,3.0f));
            granuleData[i].Velocity = Vector3.zero;
            granuleData[i].AngularVelocity = Vector3.zero;
            granuleData[i].Rotation = Quaternion.identity;
            for(int j = 0; j < 4; j++)
            {
                particlePositions[i * 4 + j] = granuleData[i].Position + tetrahedronVertices[j] * sandRadius / 2;
                particleVelocities[i * 4 + j] = granuleData[i].Velocity / 10;
            }
        }
        
        // //TEST
        // granuleData[0].Position = new Vector3(0, 4.0f, 0);
        // granuleData[0].Velocity =new Vector3(0,0,0);
        
        _particlePositionBuffer = new ComputeBuffer(particleCount*2, sizeof(float) * 3);
        _particleVelocityBuffer = new ComputeBuffer(particleCount*2, sizeof(float) * 3);
        _granuleDataBuffer = new ComputeBuffer(granuleCount*2, GranuleDataType.GetSize());
        
        _particlePositionBuffer.SetData(particlePositions);
        _particleVelocityBuffer.SetData(particleVelocities);
        _granuleDataBuffer.SetData(granuleData);
        

        _granuleInertiaReferenceTensor = CalculateRefererenceInertiaTensor();
        

        _granuleDebugDataBuffer = new ComputeBuffer(granuleCount, GranuleDebugDataType.GetSize());
        
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
    }

    private void Start()
    {
        Setup();
    }

    private void FixedUpdate()
    {
        computeShader.SetBuffer(_kernel, _particlePositionBufferId, _particlePositionBuffer);
        computeShader.SetBuffer(_kernel, _particleVelocityBufferId, _particleVelocityBuffer);
        computeShader.SetBuffer(_kernel, _granuleDataBufferId, _granuleDataBuffer);
        if(ParticelRegisterManager.Instance.rigidBodyParticleDatas.Count > 0) computeShader.SetBuffer(_kernel, _RigidBodyParticleBufferId, ParticelRegisterManager.Instance._particleBuffer);
        computeShader.SetBuffer(_kernel, _granuleDebugDataBufferId, _granuleDebugDataBuffer);
        if(PlaneRegisterManager.Instance.planeDatas.Count > 0) computeShader.SetBuffer(_kernel, _planeDataBufferId, PlaneRegisterManager.Instance._planeBuffer);
        
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
        
        
        computeShader.Dispatch(_kernel, Math.Max(1, particleCount / 64), 1, 1);
        
        
        _bufferIndexBegin = 1 - _bufferIndexBegin;
        
        DebugGranuleBuffer();
    }

    private void Update()
    {
        material.SetBuffer("_ParticlePositionBuffer", _particlePositionBuffer);//只有_ParticlePositionBuffer需要传递给材质
        material.SetInt("_BufferBeginIndex", _bufferIndexBegin*particleCount);
        Graphics.RenderMeshIndirect(_renderParams, mesh, _commandBuffer, 1);
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
