using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using Random = UnityEngine.Random;



public class SandSimulation : MonoBehaviour
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
    [Tooltip("沙子弹性恢复系数")]
    public float elasticityRestoringCoefficient = 0.1f;
    [Tooltip("接触时间")] 
    public float contactTime = 0.02f;
    [Tooltip("摩擦系数")]
    public float frictionCoefficient = 0.5f;
    [Tooltip("速度阻尼系数")]
    public float velocityDampingCoefficient = 0.5f;
    
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
    private float _viscousDampingCoefficient;
    private float _elasticityRestoringCoefficient;
    private int _bufferIndexBegin;                     //取值为0或1，用于交替读写两个缓冲区
    
    //ids
    private int _particlePositionBufferId;
    private int _particleVelocityBufferId;
    private int _granuleDataBufferId;
    private int _granuleCountId;
    private int _particleCountId;
    private int _particleMassId;
    private int _deltaTimeId;
    private int _particleRadiusId;
    private int _viscousDampingCoefficientId;
    private int _elasticityRestoringCoefficientId;
    private int _frictionCoefficientId;
    private int _velocityDampingCoefficientId;
    private int _bufferIndexBeginId;

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
    
    void SetupCommandBuffer()
    {
        _commandBuffer = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments, 1, GraphicsBuffer.IndirectDrawIndexedArgs.size);
        _commandData = new GraphicsBuffer.IndirectDrawIndexedArgs[1];
        _commandData[0].indexCountPerInstance = mesh.GetIndexCount(0);
        _commandData[0].instanceCount = (uint)particleCount;
        _commandBuffer.SetData(_commandData);
        //设置Scale
        material.SetFloat("_Scale", sandRadius);
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
        _viscousDampingCoefficientId = Shader.PropertyToID("_ViscousDampingCoefficient");
        _elasticityRestoringCoefficientId = Shader.PropertyToID("_ElasticityRestoringCoefficient");
        _frictionCoefficientId = Shader.PropertyToID("_FrictionCoefficient");
        _velocityDampingCoefficientId = Shader.PropertyToID("_VelocityDampingCoefficient");
        _particleCountId = Shader.PropertyToID("_ParticleCount");
        _bufferIndexBeginId = Shader.PropertyToID("_bufferIndexBegin");
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
            granuleData[i].Position = new Vector3(0,Random.Range(0.0f,2.0f),0);
            granuleData[i].Velocity = Vector3.zero;
            granuleData[i].AngularVelocity = Vector3.zero;
            granuleData[i].Rotation = Quaternion.identity;
            for(int j = 0; j < 4; j++)
            {
                particlePositions[i * 4 + j] = granuleData[i].Position + tetrahedronVertices[j] * sandRadius / 2;
                particleVelocities[i * 4 + j] = granuleData[i].Velocity / 10;
            }
        }
        
        _particlePositionBuffer = new ComputeBuffer(particleCount*2, sizeof(float) * 3);
        _particleVelocityBuffer = new ComputeBuffer(particleCount*2, sizeof(float) * 3);
        _granuleDataBuffer = new ComputeBuffer(granuleCount*2, GranuleDataType.GetSize());
        
        _particlePositionBuffer.SetData(particlePositions);
        _particleVelocityBuffer.SetData(particleVelocities);
        _granuleDataBuffer.SetData(granuleData);


        var m_eff = particleMass / 2;
        _viscousDampingCoefficient = 2 * m_eff * (-Mathf.Log(elasticityRestoringCoefficient)) / contactTime;
        _elasticityRestoringCoefficient = m_eff/ (contactTime * contactTime)* Mathf.Log(elasticityRestoringCoefficient*elasticityRestoringCoefficient+Mathf.PI* Mathf.PI);

        Debug.Log("viscousDampingCoefficient: " + _viscousDampingCoefficient);
        Debug.Log("elasticityRestoringCoefficient: " + _elasticityRestoringCoefficient);

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
        
        computeShader.SetInt(_granuleCountId, granuleCount);
        computeShader.SetFloat(_particleMassId, particleMass);
        computeShader.SetFloat(_deltaTimeId, Time.fixedDeltaTime);
        computeShader.SetFloat(_particleRadiusId, sandRadius);  
        computeShader.SetFloat(_viscousDampingCoefficientId, _viscousDampingCoefficient);
        computeShader.SetFloat(_elasticityRestoringCoefficientId, _elasticityRestoringCoefficient);
        computeShader.SetFloat(_frictionCoefficientId, frictionCoefficient);
        computeShader.SetFloat(_velocityDampingCoefficientId, velocityDampingCoefficient);
        computeShader.SetInt(_particleCountId, particleCount);
        computeShader.SetInt(_bufferIndexBeginId, _bufferIndexBegin);
        computeShader.Dispatch(_kernel, Math.Max(1, particleCount / 64), 1, 1);
        
        _bufferIndexBegin = 1 - _bufferIndexBegin;
    }

    private void Update()
    {
        material.SetBuffer("_ParticlePositionBuffer", _particlePositionBuffer);//只有_ParticlePositionBuffer需要传递给材质
        material.SetInt("_BufferBeginIndex", _bufferIndexBegin*particleCount);
        Graphics.RenderMeshIndirect(_renderParams, mesh, _commandBuffer, 1);
        //DebugParticleBuffer();
    }

    void DebugParticleBuffer()
    {
        Vector3[] particlePositions = new Vector3[particleCount];
        _particlePositionBuffer.GetData(particlePositions);
        for(int i = 0; i < particleCount; i++)
        {
            Debug.Log(particlePositions[i]);
        }
    }
}
