using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ParticelRegisterManager : MonoBehaviour
{
    private static ParticelRegisterManager _instance;
    public static ParticelRegisterManager Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindObjectOfType<ParticelRegisterManager>();
            }
            return _instance;
        }
    }
    
    public Material material;               //渲染粒子的材质
    public Mesh mesh;
    private RenderParams _renderParams;
    private GraphicsBuffer _commandBuffer;
    private GraphicsBuffer.IndirectDrawIndexedArgs[] _commandData;
    public ComputeBuffer _particleBuffer;   //粒子数据
    

    public struct RigidBodyParticleData
    {
        public Vector3 position;//这里为了简单，只有位置信息
    }
    
    public List<RigidBodyParticleData> rigidBodyParticleDatas = new List<RigidBodyParticleData>();
    
    private void Awake()
    {
        _commandBuffer = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments, 1, GraphicsBuffer.IndirectDrawIndexedArgs.size);
        _commandData = new GraphicsBuffer.IndirectDrawIndexedArgs[1];
        _commandData[0].indexCountPerInstance = mesh.GetIndexCount(0);
        //_commandData[0].instanceCount = (uint)particleCount;

        material.SetFloat("_Radius", 0.1f);
        _renderParams = new RenderParams(material);
        _renderParams.worldBounds = new Bounds(Vector3.zero, Vector3.one * 100);
    }

    private void Start()
    {
        _commandData[0].instanceCount = (uint)rigidBodyParticleDatas.Count;
        _commandBuffer.SetData(_commandData);
        SetupParticles();
    }


    void SetupParticles()
    {
        _particleBuffer = new ComputeBuffer(rigidBodyParticleDatas.Count, sizeof(float) * 3);
        _particleBuffer.SetData(rigidBodyParticleDatas);
    }
    
    private void OnDestroy()
    {
        _commandBuffer?.Release();
        _particleBuffer?.Release();
    }

    private void Update()
    {
        /*material.SetBuffer("_ParticlePositionBuffer",_particleBuffer);
        material.SetInt("_BufferBeginIndex",0);
        Graphics.DrawMeshInstancedIndirect(mesh, 0, material, _renderParams.worldBounds, _commandBuffer);*/
    }
}
