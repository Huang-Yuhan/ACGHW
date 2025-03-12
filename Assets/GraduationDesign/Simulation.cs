#define DEBUG_APPEND
#undef DEBUG_APPEND 

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;


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

        public float viscousDampingCoefficient=0.5f;
        public float elasticRestorationCoefficient=0.5f;
        public float frictionCoefficient=0.5f;
        public float tangentialStiffnessCoefficient=0.5f;
        
        [Header("沙粒渲染")]
        public Material material;
    
        [Header("基础设置")] 
        public float deltaTime;
    
        
        //---------------一些提前设置---------------//
        
        /// <summary>
        /// https://zh.wikipedia.org/wiki/%E6%AD%A3%E5%9B%9B%E9%9D%A2%E9%AB%94
        /// 以正四面体的中心作为原点建立三维直角坐标系的话，棱长a=2的正四面体的顶点坐标
        /// </summary>
        private readonly Vector3[] _tetrahedronVertices = new Vector3[]
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
            public uint ParticleIndexBegin;
            public uint ParticleIndexEnd;
            public uint InitialInertiaTensorIndex;
            public static int GetSize()
            {
                return sizeof(float) * 3 * 3 + sizeof(float) * 4 + sizeof(uint) * 3;
            }
        }
        
        //-----------------沙粒模拟-----------------//
        private int _currentSandCount;                      //当前一共的沙粒数量
        private int _currentGranuleCount=>_currentSandCount; //当前一共的颗粒数量
        private int _currentParticleCount=>_currentSandCount*4; //当前一共的粒子数量
        private int _bufferIndexBegin;
        public List<Matrix4x4> inertia_tensor_list;
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
            ComputeBuffer _planeDataBuffer = new ComputeBuffer(PlaneRegister.plane_data.Count,PlaneRegister.plane_data_type.GetSize());
            ComputeBuffer _inertia_tensor_rw_structured_buffer=new ComputeBuffer(inertia_tensor_list.Count,sizeof(float) * 4 * 4);
            ComputeBuffer _particle_index_to_granule_index_rw_structured_buffer=new ComputeBuffer(MaxParticleCount,sizeof(uint));
            ComputeBuffer _particle_initial_offset_rw_structured_buffer=new ComputeBuffer(MaxParticleCount,sizeof(float)*3);
            
            #if DEBUG_APPEND
            ComputeBuffer _debug_append_structured_buffer = new ComputeBuffer(100,sizeof(float)*3,ComputeBufferType.Append);
            #endif
            //---------------Add Compute Buffer---------------//
            AddComputeBuffer("particle_position_rw_structured_buffer",_particlePositionBuffer);
            AddComputeBuffer("particle_velocity_rw_structured_buffer",_particleVelocityBuffer);
            AddComputeBuffer("granule_data_rw_structured_buffer",_granuleDataBuffer);
            AddComputeBuffer("plane_data_rw_structured_buffer",_planeDataBuffer);
            AddComputeBuffer("inertia_tensor_rw_structured_buffer",_inertia_tensor_rw_structured_buffer);
            AddComputeBuffer("particle_index_to_granule_index_rw_structured_buffer",_particle_index_to_granule_index_rw_structured_buffer);
            AddComputeBuffer("particle_initial_offset_rw_structured_buffer",_particle_initial_offset_rw_structured_buffer);
            #if DEBUG_APPEND
            AddComputeBuffer("debug_append_structured_buffer",_debug_append_structured_buffer);
            #endif
            
            
            //---------------Add Kernels---------------//
            AddKernel("CSMain");
        }

        private void InitRender()
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

        private void InitData()
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

        private void InitDynamicAddData()
        {
            throw new NotImplementedException();
        }
        private void InitNonDynamicAddData()
        {
            //---------------初始化Sand Granule Data---------------//
            GranuleDataType[] granuleData = new GranuleDataType[_buffers["granule_data_rw_structured_buffer"].count];
            Vector3[] particlePosition = new Vector3[_buffers["particle_position_rw_structured_buffer"].count];
            Vector3[] particleVelocity = new Vector3[_buffers["particle_velocity_rw_structured_buffer"].count];
            uint[] particleIndexToGranuleIndex = new uint[_buffers["particle_index_to_granule_index_rw_structured_buffer"].count];
            Vector3[] particleInitialOffset = new Vector3[_buffers["particle_initial_offset_rw_structured_buffer"].count];
            for (int i = 0; i < granuleData.Length; i++)
            {
                granuleData[i].Position = new Vector3(Random.Range(-9f, 9f), Random.Range(1f,5f), Random.Range(-9f, 9f));
                granuleData[i].Velocity = Random.onUnitSphere;
                granuleData[i].AngularVelocity = Vector3.zero;
                granuleData[i].Rotation = Quaternion.identity;
                granuleData[i].ParticleIndexBegin = (uint)((i * 4)%MaxParticleCount);
                granuleData[i].ParticleIndexEnd = (uint)((i * 4 + 3)%MaxParticleCount);
                granuleData[i].InitialInertiaTensorIndex = 0;           //因为沙粒的模型都是一样的。所以这里直接设置为0
                for(int j=0;j<4;j++)
                {
                    uint index = (uint)(i * 4 + j);
                    particlePosition[index] = granuleData[i].Position + _tetrahedronVertices[j] * particleRadius;
                    particleVelocity[index] = granuleData[i].Velocity;          //TODO:这里严格来说是有问题的，需要考虑Granule的角速度对粒子速度的影响
                    if(index<particleIndexToGranuleIndex.Length)
                        particleIndexToGranuleIndex[index] = (uint)i;
                    if(index<particleInitialOffset.Length)
                        particleInitialOffset[index] = _tetrahedronVertices[j] * particleRadius;
                }
            }

            _currentSandCount = maxSandCount;
            _bufferIndexBegin = 0;
            
            //---------------设置Buffer Data---------------//
            _buffers["granule_data_rw_structured_buffer"].SetData(granuleData);
            _buffers["particle_position_rw_structured_buffer"].SetData(particlePosition);
            _buffers["particle_velocity_rw_structured_buffer"].SetData(particleVelocity);
            _buffers["plane_data_rw_structured_buffer"].SetData(PlaneRegister.plane_data.ToArray());
            _buffers["inertia_tensor_rw_structured_buffer"].SetData(inertia_tensor_list.ToArray());
            _buffers["particle_index_to_granule_index_rw_structured_buffer"].SetData(particleIndexToGranuleIndex);
            _buffers["particle_initial_offset_rw_structured_buffer"].SetData(particleInitialOffset);
            
            
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
            AddId("k_d");
            AddId("k_r");
            AddId("k_t");
            AddId("mu");
            AddId("plane_count");
            AddId("I_inverse_initial");

        }

        private void Awake()
        {
            inertia_tensor_list.Add(CalculateRefererenceInertiaTensor().inverse);
        }

        private void Start()
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
            Debug.LogFormat("Add Compute Buffer: {0},length:{1},size:{2}",name,buffer.count,buffer.stride);
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
            cs.SetFloat(shaderParameterIds["k_d"], viscousDampingCoefficient);
            cs.SetFloat(shaderParameterIds["k_r"], elasticRestorationCoefficient);
            cs.SetFloat(shaderParameterIds["k_t"], tangentialStiffnessCoefficient);
            cs.SetFloat(shaderParameterIds["mu"], frictionCoefficient);
            cs.SetInt(shaderParameterIds["plane_count"], PlaneRegister.plane_data.Count);
            
            //-----------------设置Buffer-----------------//
            
            //CSMain 
            ComputeShaderSetBuffer("CSMain","particle_position_rw_structured_buffer");
            ComputeShaderSetBuffer("CSMain","particle_velocity_rw_structured_buffer");
            ComputeShaderSetBuffer("CSMain","granule_data_rw_structured_buffer");
            ComputeShaderSetBuffer("CSMain","plane_data_rw_structured_buffer");
            ComputeShaderSetBuffer("CSMain","inertia_tensor_rw_structured_buffer");
            ComputeShaderSetBuffer("CSMain","particle_index_to_granule_index_rw_structured_buffer");
            ComputeShaderSetBuffer("CSMain","particle_initial_offset_rw_structured_buffer");
            #if DEBUG_APPEND
            ComputeShaderSetBuffer("CSMain","debug_append_structured_buffer");
            #endif
            
            _bufferIndexBegin=1-_bufferIndexBegin;
            
            
            cs.Dispatch(_kernels["CSMain"],Mathf.CeilToInt(_currentGranuleCount/32f), 1, 1);
            
            //DEBUG:
            // GranuleDataType[] granuleData = new GranuleDataType[MaxGranuleCount * 2];
            // _buffers["granule_data_rw_structured_buffer"].GetData(granuleData);
            // for(int i=0;i<granuleData.Length;i++)
            //     Debug.LogFormat("position:{0},velocity:{1},angularVelocity:{2},rotation:{3}",granuleData[i].Position,granuleData[i].Velocity,granuleData[i].AngularVelocity,granuleData[i].Rotation);
            
            #if DEBUG_APPEND
            ComputeBuffer debug_append_count_buffer = new ComputeBuffer(1,sizeof(int),ComputeBufferType.Raw);
            ComputeBuffer.CopyCount(_buffers["debug_append_structured_buffer"],debug_append_count_buffer,0);
            int[] debug_append_count = new int[1];
            debug_append_count_buffer.GetData(debug_append_count);
            if(debug_append_count[0]>0)
            {
                Debug.LogFormat("debug buffer 中数据有{0}个",debug_append_count[0]);
                Debug.Log("debug buffer中的数据为:");
                Vector3[] debug_append_data=new Vector3[debug_append_count[0]];
                _buffers["debug_append_structured_buffer"].GetData(debug_append_data);
                for(int i=0;i<debug_append_data.Length;i+=3)
                    Debug.LogFormat("granule粒子位置:{0},平面粒子位置:{1},二者接触力计算结果为:{2}",debug_append_data[i],debug_append_data[i+1],debug_append_data[i+2]);
            }
            //清空Append Buffer
            _buffers["debug_append_structured_buffer"].SetCounterValue(0);
            debug_append_count_buffer.Release();
            #endif

        }
        Matrix4x4 CalculateRefererenceInertiaTensor()
        {
            Mesh sphereMesh = mesh;
            Vector3[] vertices = sphereMesh.vertices;
            Matrix4x4 I_ref = Matrix4x4.zero;
            float vertexMass = particleMass;
            for (int i = 0; i < _tetrahedronVertices.Length; i++)
            {
                I_ref[0, 0] += vertexMass * _tetrahedronVertices[i].sqrMagnitude;
                I_ref[1, 1] += vertexMass * _tetrahedronVertices[i].sqrMagnitude;
                I_ref[2, 2] += vertexMass * _tetrahedronVertices[i].sqrMagnitude;
                I_ref[0, 0] -= vertexMass * _tetrahedronVertices[i][0] * _tetrahedronVertices[i][0];
                I_ref[0, 1] -= vertexMass * _tetrahedronVertices[i][0] * _tetrahedronVertices[i][1];
                I_ref[0, 2] -= vertexMass * _tetrahedronVertices[i][0] * _tetrahedronVertices[i][2];
                I_ref[1, 0] -= vertexMass * _tetrahedronVertices[i][1] * _tetrahedronVertices[i][0];
                I_ref[1, 1] -= vertexMass * _tetrahedronVertices[i][1] * _tetrahedronVertices[i][1];
                I_ref[1, 2] -= vertexMass * _tetrahedronVertices[i][1] * _tetrahedronVertices[i][2];
                I_ref[2, 0] -= vertexMass * _tetrahedronVertices[i][2] * _tetrahedronVertices[i][0];
                I_ref[2, 1] -= vertexMass * _tetrahedronVertices[i][2] * _tetrahedronVertices[i][1];
                I_ref[2, 2] -= vertexMass * _tetrahedronVertices[i][2] * _tetrahedronVertices[i][2];
            }
            I_ref[3, 3] = 1;                
            return I_ref;
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

