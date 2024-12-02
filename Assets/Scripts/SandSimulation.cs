using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
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
    private ComputeBuffer _ParticlePositionBuffer;
    private ComputeBuffer _ParticleVelocityBuffer;
    private ComputeBuffer _GranuleDataBuffer;
    private RenderParams _RenderParams;
    private GraphicsBuffer _CommandBuffer;
    GraphicsBuffer.IndirectDrawIndexedArgs[] commandData;

    struct GranuleDataType
    {
        public Vector3 position;
        public Vector3 velocity;
        public Vector3 angularVelocity;
        public Quaternion rotation;
        public static int GetSize()
        {
            return sizeof(float) * 3 * 3 + sizeof(float) * 4;
        }
    }
    
    void SetupCommandBuffer()
    {
        _CommandBuffer = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments, 1, GraphicsBuffer.IndirectDrawIndexedArgs.size);
        commandData = new GraphicsBuffer.IndirectDrawIndexedArgs[1];
        commandData[0].indexCountPerInstance = mesh.GetIndexCount(0);
        commandData[0].instanceCount = (uint)particleCount;
        _CommandBuffer.SetData(commandData);
        //设置Scale
        material.SetFloat("_Scale", sandRadius);
        _RenderParams = new RenderParams(material);
        _RenderParams.worldBounds = new Bounds(Vector3.zero, 100 * Vector3.one);//设定边界
        
    }
    
    void SetupSimulation()
    {
        _kernel = computeShader.FindKernel("CSMain");
        Vector3[] particlePositions = new Vector3[particleCount];
        Vector3[] particleVelocities = new Vector3[particleCount];
        GranuleDataType[] granuleData = new GranuleDataType[granuleCount];
        for(int i = 0; i < granuleCount; i++)
        {
            granuleData[i].position = new Vector3(Random.Range(-10, 10), Random.Range(-10, 10), Random.Range(-10, 10));
            granuleData[i].velocity = Random.insideUnitSphere;
            for(int j = 0; j < 4; j++)
            {
                particlePositions[i * 4 + j] = granuleData[i].position + tetrahedronVertices[j] * sandRadius / 2;
                particleVelocities[i * 4 + j] = granuleData[i].velocity / 10;
            }
        }
        
        _ParticlePositionBuffer = new ComputeBuffer(particleCount, sizeof(float) * 3);
        _ParticleVelocityBuffer = new ComputeBuffer(particleCount, sizeof(float) * 3);
        _GranuleDataBuffer = new ComputeBuffer(granuleCount, GranuleDataType.GetSize());
        
        _ParticlePositionBuffer.SetData(particlePositions);
        _ParticleVelocityBuffer.SetData(particleVelocities);
        _GranuleDataBuffer.SetData(granuleData);
    }
    
    void Setup()
    {
        SetupCommandBuffer();
        SetupSimulation();
    }

    private void OnDestroy()
    {
        _ParticlePositionBuffer?.Release();
        _ParticleVelocityBuffer?.Release();
        _GranuleDataBuffer?.Release();
        _CommandBuffer?.Release();
    }

    private void Start()
    {
        Setup();
    }

    private void Update()
    {
        material.SetBuffer("_ParticlePositionBuffer", _ParticlePositionBuffer);//只有_ParticlePositionBuffer需要传递给材质
        computeShader.SetBuffer(_kernel, "_ParticlePositionBuffer", _ParticlePositionBuffer);
        computeShader.SetBuffer(_kernel, "_ParticleVelocityBuffer", _ParticleVelocityBuffer);
        computeShader.SetBuffer(_kernel, "_GranuleDataBuffer", _GranuleDataBuffer);
        computeShader.Dispatch(_kernel, particleCount / 64, 1, 1);
        Graphics.RenderMeshIndirect(_RenderParams, mesh, _CommandBuffer, 1);
    }
}
