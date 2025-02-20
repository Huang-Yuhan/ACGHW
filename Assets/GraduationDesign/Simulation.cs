using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace GraduationDesign
{
    public class Simulation : MonoBehaviour
    {
        [Header("沙粒属性")]
        public int maxSandCount;
        public int MaxGranuleCount => maxSandCount;
        public int MaxParticleCount => maxSandCount * 4;
        public float particleRadius;
        public float particleMass;
        public Mesh mesh;
    
        [Header("沙粒模拟")]
        public ComputeShader cs;

        public bool isDynamicAdding;
        
        [Header("沙粒渲染")]
        public Material material;
    
        [Header("基础设置")] 
        public float deltaTime;
    
        
        //---------------一些提前设置---------------//
        
        /// <summary>
        /// https://zh.wikipedia.org/wiki/%E6%AD%A3%E5%9B%9B%E9%9D%A2%E9%AB%94
        /// 以正四面体的中心作为原点建立三维直角坐标系的话，棱长a=2的正四面体的顶点坐标
        /// </summary>
        private Vector3[] _tetrahedronVertices = new Vector3[]
        {
            new Vector3(1,0,-1/(float)Math.Sqrt(2)),
            new Vector3(-1,0,-1/(float)Math.Sqrt(2)),
            new Vector3(0,1,1/(float)Math.Sqrt(2)),
            new Vector3(0,-1,1/(float)Math.Sqrt(2))
        };
        
        private struct GranuleDataType
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
        
        //-----------------沙粒模拟-----------------//
        private int _currentSandCount;                      //当前一共的沙粒数量
        private int _currentGranuleCount=>_currentSandCount; //当前一共的颗粒数量
        private int _currentParticleCount=>_currentSandCount*4; //当前一共的粒子数量
        private int _bufferIndexBegin;
        
        //-----------------沙粒渲染-----------------//
        private RenderParams _renderParams;
        private GraphicsBuffer _commandBuffer;
        private GraphicsBuffer.IndirectDrawIndexedArgs[] _commandData;
        
        //-----------------Kernels-----------------//
        Dictionary<string,int> _kernels = new Dictionary<string, int>();
        
        //-----------------Compute Buffers-----------------//
        
        Dictionary<string,int> shaderParameterIds = new Dictionary<string, int>();
        Dictionary<string,ComputeBuffer> _buffers = new Dictionary<string, ComputeBuffer>();        //因为ComputeBuffer需要在OnDestroy中释放，所以这里用Dictionary存储可以方便释放


        void InitSimulation()
        {
            //---------------初始化Compute Buffer---------------//
            ComputeBuffer _particlePositionBuffer = new ComputeBuffer(MaxParticleCount*2,sizeof(float)*3);        //粒子位置，*2是类似于OpenGL中Transform Feedback的双缓冲
            ComputeBuffer _particleVelocityBuffer = new ComputeBuffer(MaxParticleCount*2,sizeof(float)*3);        //粒子速度
            ComputeBuffer _granuleDataBuffer = new ComputeBuffer(MaxGranuleCount*2,GranuleDataType.GetSize());      //颗粒数据
            
            //---------------Add Compute Buffer---------------//
            AddComputeBuffer("particle_position_rw_structured_buffer",_particlePositionBuffer);
            AddComputeBuffer("particle_velocity_rw_structured_buffer",_particleVelocityBuffer);
            AddComputeBuffer("granule_data_rw_structured_buffer",_granuleDataBuffer);
            
            
            //---------------Add Kernels---------------//
            AddKernel("CSMain");
        }

        void InitRender()
        {
            _commandBuffer = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments, 1, GraphicsBuffer.IndirectDrawIndexedArgs.size);
            _commandData = new GraphicsBuffer.IndirectDrawIndexedArgs[1];
            _commandData[0].indexCountPerInstance = mesh.GetIndexCount(0);
            _commandData[0].instanceCount = (uint)_currentParticleCount;
            _commandBuffer.SetData(_commandData);
            //设置Scale
            material.SetFloat("_Radius", particleRadius);
            _renderParams = new RenderParams(material);
            _renderParams.worldBounds = new Bounds(Vector3.zero, 100 * Vector3.one);//设定边界
        }

        void InitData()
        {
            if(isDynamicAdding)
            {
                InitDynamicAddData();
            }
            else
            {
                InitNonDynamicAddData();
            }
        }

        void InitDynamicAddData()
        {
            throw new NotImplementedException();
        }
        void InitNonDynamicAddData()
        {
            //---------------初始化Granule Data---------------//
            GranuleDataType[] granuleData = new GranuleDataType[_buffers["granule_data_rw_structured_buffer"].count];
            Vector3[] particlePosition = new Vector3[_buffers["particle_position_rw_structured_buffer"].count];
            Vector3[] particleVelocity = new Vector3[_buffers["particle_velocity_rw_structured_buffer"].count];
            for (int i = 0; i < granuleData.Length; i++)
            {
                granuleData[i].Position = new Vector3(0, 5, 0);
                granuleData[i].Velocity = Vector3.zero;
                granuleData[i].AngularVelocity = Vector3.zero;
                granuleData[i].Rotation = Quaternion.identity;
                for(int j=0;j<4;j++)
                {
                    particlePosition[i*4+j] = granuleData[i].Position + _tetrahedronVertices[j] * particleRadius;
                    particleVelocity[i*4+j] = granuleData[i].Velocity;          //TODO:这里严格来说是有问题的，需要考虑Granule的角速度对粒子速度的影响
                }
            }

            _currentSandCount = maxSandCount;
            _bufferIndexBegin = 0;
            
            //---------------设置Buffer Data---------------//
            _buffers["granule_data_rw_structured_buffer"].SetData(granuleData);
            _buffers["particle_position_rw_structured_buffer"].SetData(particlePosition);
            _buffers["particle_velocity_rw_structured_buffer"].SetData(particleVelocity);
            
            //---------------添加参数ID---------------//
            AddId("delta_time");
            AddId("current_granule_count");
            AddId("max_granule_count");
            AddId("current_particle_count");
            AddId("max_particle_count");
            AddId("particle_mass");
            AddId("particle_radius");
            AddId("consume_granule_count");
            AddId("buffer_index_begin");

        }

        private void Awake()
        {
            InitSimulation();
            InitData();
            InitRender();
        }


        private void AddKernel(string name)
        {
            int kernel = cs.FindKernel(name);
            _kernels.Add(name, kernel);
        }
        
        private void AddComputeBuffer(string name, ComputeBuffer buffer)
        {
            int id=Shader.PropertyToID(name);
            shaderParameterIds.Add(name, id);
            _buffers.Add(name, buffer);
        }

        private void AddId(string name)
        {
            int id=Shader.PropertyToID(name);
            shaderParameterIds[name]=id;
        }

        private void ComputeShaderSetBuffer(string kernel_name, string buffer_name)
        {
            cs.SetBuffer(_kernels[kernel_name], shaderParameterIds[buffer_name], _buffers[buffer_name]);
        }
        
        private void ReleaseComputeBuffers()
        {
            foreach (var buffer in _buffers)
            {
                buffer.Value.Release();
            }
            _buffers.Clear();
        }
        
        private void ReleaseGraphicsBuffers()
        {
            _commandBuffer.Release();
        }

        private void OnDestroy()
        {
            ReleaseComputeBuffers();
            ReleaseGraphicsBuffers();
        }

        void ProcessRender()
        {
            material.SetBuffer("_ParticlePositionBuffer", _buffers["particle_position_rw_structured_buffer"]);
            material.SetInt("_BufferBeginIndex", _bufferIndexBegin*_currentParticleCount);
            Graphics.RenderMeshIndirect(_renderParams, mesh, _commandBuffer, 1);                                                //渲染
        }

        void ProcessSimulation()
        {
            //-----------------设置参数-----------------//
            cs.SetFloat(shaderParameterIds["delta_time"], deltaTime);
            cs.SetInt(shaderParameterIds["current_granule_count"], _currentGranuleCount);
            cs.SetInt(shaderParameterIds["max_granule_count"], MaxGranuleCount);
            cs.SetInt(shaderParameterIds["current_particle_count"], _currentParticleCount);
            cs.SetInt(shaderParameterIds["max_particle_count"], MaxParticleCount);
            cs.SetFloat(shaderParameterIds["particle_mass"], particleMass);
            cs.SetFloat(shaderParameterIds["particle_radius"], particleRadius);
            cs.SetInt(shaderParameterIds["buffer_index_begin"], _bufferIndexBegin);
            //-----------------设置Buffer-----------------//
            
            //CSMain 
            ComputeShaderSetBuffer("CSMain","particle_position_rw_structured_buffer");
            ComputeShaderSetBuffer("CSMain","particle_velocity_rw_structured_buffer");
            ComputeShaderSetBuffer("CSMain","granule_data_rw_structured_buffer");
            
            _bufferIndexBegin=1-_bufferIndexBegin;
            
            
            cs.Dispatch(_kernels["CSMain"],Mathf.CeilToInt(_currentGranuleCount/32f), 1, 1);
            
            //DEBUG:
            GranuleDataType[] granuleData = new GranuleDataType[MaxGranuleCount * 2];
            _buffers["granule_data_rw_structured_buffer"].GetData(granuleData);
            for(int i=0;i<granuleData.Length;i++)
            Debug.LogFormat("position:{0},velocity:{1},angularVelocity:{2},rotation:{3}",granuleData[i].Position,granuleData[i].Velocity,granuleData[i].AngularVelocity,granuleData[i].Rotation);

        }

        private void Update()
        {
            ProcessRender();
        }

        private void FixedUpdate()
        {
            ProcessSimulation();
        }
    }

}

