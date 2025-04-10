#define DEBUG_APPEND
#undef DEBUG_APPEND 
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;
using Random = UnityEngine.Random;
using GPUSorting;
using GPUSorting.Runtime;
using UnityEngine.Assertions;
using UnityEngine.Rendering;


namespace GraduationDesign
{
    public class Simulation : MonoBehaviour
    {
        [Header("沙粒属性")]
        public int maxSandCount;
        public int MaxSandGranuleCount => maxSandCount;
        public int MaxGranuleCount=>MaxSandGranuleCount+RigidBodyRegister.RigidBodyData.Count;
        public int MaxSandParticleCount => maxSandCount * 4;
        public int MaxParticleCount => MaxSandParticleCount + RigidBodyRegister.ParticleSum;
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

        [Header("邻域搜索加速")] 
        public Vector3 gridCellSize;

        public Vector3 gridOrigin;
        public Vector3 gridLength;
        public ComputeShader sortingShader;
        private GPUSorting.Runtime.DeviceRadixSort sorter;
        private ComputeBuffer temp0Buffer;
        private ComputeBuffer temp1Buffer;
        private ComputeBuffer temp2Buffer;
        private ComputeBuffer temp3Buffer;
    
        
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
        
        public struct GranuleDataType
        {
            public Vector3 Position;
            public Vector3 Velocity;
            public Vector3 AngularVelocity;
            public Quaternion Rotation;
            public uint ParticleIndexBegin;
            public uint ParticleIndexEnd;
            public uint InitialInertiaTensorIndex;
            public float ParticleRadius;
            public float ParticleMass;
            public static int GetSize()
            {
                return sizeof(float) * 3 * 3 + sizeof(float) * 4 + sizeof(uint) * 3 + sizeof(float)+sizeof(float);
            }
        }
        
        //-----------------沙粒模拟-----------------//
        private int _currentSandCount;                      //当前一共的沙粒数量
        private int _currentGranuleCount=>_currentSandCount+RigidBodyRegister.RigidBodyData.Count; //当前一共的颗粒数量
        private int _currentParticleCount=>_currentSandCount*4+RigidBodyRegister.ParticleSum; //当前一共的粒子数量
        private int _bufferIndexBegin;
        public List<Matrix4x4> inertia_tensor_list;
        const float THREAD_GROUP_SIZE_X = 64;
        //-----------------沙粒渲染-----------------//
        private RenderParams _renderParams;
        private GraphicsBuffer _commandBuffer;
        private GraphicsBuffer.IndirectDrawIndexedArgs[] _commandData;
        //-----------------GPU邻域搜索加速-----------//
        private Vector3Int _gridResolution
        {
            get
            {
                return new Vector3Int((int)(gridLength.x/gridCellSize.x),
                    (int)(gridLength.y/gridCellSize.y),
                    (int)(gridLength.z/gridCellSize.z));
            }
        }
        
        //-----------------Kernels-----------------//
        Dictionary<string,int> _kernels = new Dictionary<string, int>();
        
        //-----------------Compute Buffers-----------------//
        
        Dictionary<string,int> shaderParameterIds = new Dictionary<string, int>();
        Dictionary<string,ComputeBuffer> _buffers = new Dictionary<string, ComputeBuffer>();        //因为ComputeBuffer需要在OnDestroy中释放，所以这里用Dictionary存储可以方便释放


        void InitSimulation()
        {
            SetupGrid();
            //---------------初始化Compute Buffer---------------//
            ComputeBuffer _particlePositionBuffer = new ComputeBuffer(MaxParticleCount*2,sizeof(float)*3);        //粒子位置，*2是类似于OpenGL中Transform Feedback的双缓冲
            ComputeBuffer _particleVelocityBuffer = new ComputeBuffer(MaxParticleCount*2,sizeof(float)*3);        //粒子速度
            ComputeBuffer _granuleDataBuffer = new ComputeBuffer(MaxGranuleCount*2,GranuleDataType.GetSize());      //颗粒数据
            ComputeBuffer _planeDataBuffer = new ComputeBuffer(PlaneRegister.plane_data.Count,PlaneRegister.plane_data_type.GetSize());
            ComputeBuffer _inertia_tensor_rw_structured_buffer=new ComputeBuffer(1+RigidBodyRegister.RigidBodyData.Count,sizeof(float) * 4 * 4);
            ComputeBuffer _particle_index_to_granule_index_rw_structured_buffer=new ComputeBuffer(MaxParticleCount,sizeof(uint));
            ComputeBuffer _particle_initial_offset_rw_structured_buffer=new ComputeBuffer(MaxParticleCount,sizeof(float)*3);
            ComputeBuffer _particle_contact_force_rw_structured_buffer=new ComputeBuffer(MaxParticleCount,sizeof(float) * 3);
            ComputeBuffer _particle_in_grid_index_rw_structured_buffer=new ComputeBuffer(MaxParticleCount,sizeof(uint));
            ComputeBuffer _particle_in_grid_particle_index_rw_structured_buffer=new ComputeBuffer(MaxParticleCount,sizeof(uint));
            
            
            #if DEBUG_APPEND
            ComputeBuffer _debug_append_structured_buffer = new ComputeBuffer(65536,sizeof(float)*3,ComputeBufferType.Append);
            ComputeBuffer debug_append_count_buffer = new ComputeBuffer(1,sizeof(int),ComputeBufferType.Raw);
            cs.EnableKeyword("DEBUG");  
            #else
            cs.DisableKeyword("DEBUG");
            #endif
            //---------------Add Compute Buffer---------------//
            AddComputeBuffer("particle_position_rw_structured_buffer",_particlePositionBuffer);
            AddComputeBuffer("particle_velocity_rw_structured_buffer",_particleVelocityBuffer);
            AddComputeBuffer("granule_data_rw_structured_buffer",_granuleDataBuffer);
            AddComputeBuffer("plane_data_rw_structured_buffer",_planeDataBuffer);
            AddComputeBuffer("inertia_tensor_rw_structured_buffer",_inertia_tensor_rw_structured_buffer);
            AddComputeBuffer("particle_index_to_granule_index_rw_structured_buffer",_particle_index_to_granule_index_rw_structured_buffer);
            AddComputeBuffer("particle_initial_offset_rw_structured_buffer",_particle_initial_offset_rw_structured_buffer);
            AddComputeBuffer("particle_contact_force_rw_structured_buffer",_particle_contact_force_rw_structured_buffer);
            AddComputeBuffer("particle_in_grid_index_rw_structured_buffer",_particle_in_grid_index_rw_structured_buffer);
            AddComputeBuffer("particle_in_grid_particle_index_rw_structured_buffer",_particle_in_grid_particle_index_rw_structured_buffer);
            AddComputeBuffer("temp0Buffer",temp0Buffer);
            AddComputeBuffer("temp1Buffer",temp1Buffer);
            AddComputeBuffer("temp2Buffer",temp2Buffer);
            AddComputeBuffer("temp3Buffer",temp3Buffer);
            #if DEBUG_APPEND
            AddComputeBuffer("debug_append_structured_buffer",_debug_append_structured_buffer);
            AddComputeBuffer("debug_append_count_buffer",debug_append_count_buffer);
            #endif
            
            
            //---------------Add Kernels---------------//
            AddKernel("UpdateGranuleKernel");
            AddKernel("ForceCalculationKernel");
            AddKernel("UpdateParticleKernel");


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
            uint[] particleGridIndex = new uint[_buffers["particle_in_grid_index_rw_structured_buffer"].count];
            uint[] particleGridParticleIndex = new uint[_buffers["particle_in_grid_particle_index_rw_structured_buffer"].count];
            for (int i = 0; i < MaxSandGranuleCount; i++)
            {
                granuleData[i].Position =
                    new Vector3(Random.Range(-3f, 3f), Random.Range(1f, 5f), Random.Range(-3f, 3f));
                granuleData[i].Velocity = Random.onUnitSphere;

                if (i == 0)
                {
                    granuleData[i].Position = new Vector3(0, 2, 0);
                    granuleData[i].Velocity = new Vector3(0, 0, 0);
                }
                
                
                if (i == 1)
                {
                    granuleData[i].Position= new Vector3(0, 3, 0);
                    granuleData[i].Velocity = new Vector3(0, 0, 0);
                }
                
                granuleData[i].AngularVelocity = Vector3.zero;
                granuleData[i].Rotation = Quaternion.identity;
                granuleData[i].ParticleIndexBegin = (uint)((i * 4)%MaxSandParticleCount);
                granuleData[i].ParticleIndexEnd = (uint)((i * 4 + 3)%MaxSandParticleCount);
                granuleData[i].InitialInertiaTensorIndex = 0;           //因为沙粒的模型都是一样的。所以这里直接设置为0
                granuleData[i].ParticleRadius = particleRadius;
                granuleData[i].ParticleMass=particleMass;
                for(int j=0;j<4;j++)
                {
                    uint index = (uint)(i * 4 + j);
                    particlePosition[index] = granuleData[i].Position + _tetrahedronVertices[j] * particleRadius;
                    particleVelocity[index] = granuleData[i].Velocity;          
                    if(index<particleIndexToGranuleIndex.Length)
                        particleIndexToGranuleIndex[index] = (uint)i;
                    if(index<particleInitialOffset.Length)
                        particleInitialOffset[index] = _tetrahedronVertices[j] * particleRadius;
                    Vector3Int grid_index = new Vector3Int((int)((particlePosition[index].x-gridOrigin.x)/gridCellSize.x),
                        (int)((particlePosition[index].y-gridOrigin.y)/gridCellSize.y),
                        (int)((particlePosition[index].z-gridLength.z)/gridCellSize.z));
                    particleGridIndex[index] = (uint)(grid_index.x+_gridResolution.x*grid_index.y+_gridResolution.x*_gridResolution.y*grid_index.z);
                    particleGridParticleIndex[index] = index;
                }
            }

            _currentSandCount = maxSandCount;
            _bufferIndexBegin = 0;
            
            int preGranuleCount = MaxSandGranuleCount;
            int preParticleCount = MaxSandParticleCount;
            
            //---------------初始化RigidBody Data---------------//
            for (int i = 0; i < RigidBodyRegister.RigidBodyData.Count; i++)
            {
                GranuleDataType data = new GranuleDataType();
                data.Position = RigidBodyRegister.RigidBodyData[i].Position;
                data.Velocity = RigidBodyRegister.RigidBodyData[i].Velocity;
                data.AngularVelocity = RigidBodyRegister.RigidBodyData[i].AngularVelocity;
                data.Rotation = RigidBodyRegister.RigidBodyData[i].Rotation;
                data.ParticleIndexBegin = (uint)(preParticleCount);
                data.ParticleIndexEnd = (uint)(preParticleCount+RigidBodyRegister.RigidBodyData[i].RigidBodiesParticleInitialOffset.Count-1);
                data.InitialInertiaTensorIndex = (uint)(inertia_tensor_list.Count);
                data.ParticleRadius=RigidBodyRegister.RigidBodyData[i].ParticleRadius;
                data.ParticleMass=RigidBodyRegister.RigidBodyData[i].ParticleMass;
                inertia_tensor_list.Add(CalculateRefererenceInertiaTensor(RigidBodyRegister.RigidBodyData[i].RigidBodiesParticleInitialOffset).inverse);
                granuleData[preGranuleCount + i] = data;
                for (int j = 0; j < RigidBodyRegister.RigidBodyData[i].RigidBodiesParticleInitialOffset.Count; j++)
                {
                    uint index = (uint)(preParticleCount + j);
                    particlePosition[index] = RigidBodyRegister.RigidBodyData[i].Position + RigidBodyRegister.RigidBodyData[i].RigidBodiesParticleInitialOffset[j];
                    particleVelocity[index] = RigidBodyRegister.RigidBodyData[i].Velocity;
                    if(index<particleIndexToGranuleIndex.Length)
                        particleIndexToGranuleIndex[index] = (uint)(preGranuleCount+i);
                    if(index<particleInitialOffset.Length)
                        particleInitialOffset[index] = RigidBodyRegister.RigidBodyData[i].RigidBodiesParticleInitialOffset[j];
                    
                    Vector3Int grid_index = new Vector3Int((int)((particlePosition[index].x-gridOrigin.x)/gridCellSize.x),(int)((particlePosition[index].y-gridOrigin.y)/gridCellSize.y),(int)((particlePosition[index].z-gridOrigin.z)/gridCellSize.z));
                    particleGridIndex[index] = (uint)(grid_index.x+_gridResolution.x*grid_index.y+_gridResolution.x*_gridResolution.y*grid_index.z);
                    particleGridParticleIndex[index] = index;
                }
                preParticleCount += RigidBodyRegister.RigidBodyData[i].RigidBodiesParticleInitialOffset.Count;
                preGranuleCount++;
            }
            
            //---------------设置Buffer Data---------------//
            _buffers["granule_data_rw_structured_buffer"].SetData(granuleData);
            _buffers["particle_position_rw_structured_buffer"].SetData(particlePosition);
            _buffers["particle_velocity_rw_structured_buffer"].SetData(particleVelocity);
            _buffers["plane_data_rw_structured_buffer"].SetData(PlaneRegister.plane_data.ToArray());
            _buffers["inertia_tensor_rw_structured_buffer"].SetData(inertia_tensor_list.ToArray());
            _buffers["particle_index_to_granule_index_rw_structured_buffer"].SetData(particleIndexToGranuleIndex);
            _buffers["particle_initial_offset_rw_structured_buffer"].SetData(particleInitialOffset);
            _buffers["particle_in_grid_index_rw_structured_buffer"].SetData(particleGridIndex);
            _buffers["particle_in_grid_particle_index_rw_structured_buffer"].SetData(particleGridParticleIndex);
            #if DEBUG_APPEND
            _buffers["debug_append_structured_buffer"].SetCounterValue(0);
            #endif
            
            
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
            AddId("grid_size");
            AddId("grid_origin");
            AddId("buffer_index_begin_multiple_max_granule_count");
            AddId("grid_resolution");

            uint param_size = 0;
            param_size += sizeof(float)*3; //delta_time
            param_size += sizeof(int); //current_granule_count
            param_size += sizeof(int); //max_granule_count
            param_size += sizeof(int); //current_particle_count
            param_size += sizeof(int); //max_particle_count
            param_size += sizeof(float); //particle_mass
            param_size += sizeof(float); //particle_radius
            param_size += sizeof(int); //consume_granule_count
            param_size += sizeof(int); //buffer_index_begin
            param_size += sizeof(float); //k_d
            param_size += sizeof(float); //k_r
            param_size += sizeof(float); //k_t
            param_size += sizeof(float); //mu
            param_size += sizeof(int); //plane_count
            param_size += sizeof(float)*4*4; //I_inverse_initial
            param_size += sizeof(float)*3; //grid_size
            param_size += sizeof(float)*3; //grid_origin
            
        }

        private void Awake()
        {
            inertia_tensor_list.Add(CalculateRefererenceInertiaTensor(_tetrahedronVertices.ToList()).inverse);
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
            cs.SetVector(shaderParameterIds["grid_size"], gridCellSize);
            cs.SetVector(shaderParameterIds["grid_resolution"],(Vector3)_gridResolution);
            cs.SetVector(shaderParameterIds["grid_origin"], gridOrigin);
            cs.SetInt(shaderParameterIds["buffer_index_begin_multiple_max_granule_count"],_bufferIndexBegin*MaxGranuleCount);
            
            
            //-----------------GPU Sort-----------------//
            sorter.Sort(
                MaxParticleCount,
                _buffers["particle_in_grid_index_rw_structured_buffer"],
                _buffers["particle_in_grid_particle_index_rw_structured_buffer"],
                temp0Buffer,
                temp1Buffer,
                temp2Buffer,
                temp3Buffer,
                typeof(uint),
                typeof(uint),
                true
                );
            
            //显式地等待GPU完成排序
            AsyncGPUReadback.Request(_buffers["particle_in_grid_index_rw_structured_buffer"]);
            AsyncGPUReadback.Request(_buffers["particle_in_grid_particle_index_rw_structured_buffer"]);
            AsyncGPUReadback.WaitAllRequests();

#if DEBUG_APPEND
            int[] particle_in_grid_index_rw_structured_buffer=new int[_buffers["particle_in_grid_index_rw_structured_buffer"].count];
            int[] particle_in_grid_particle_index_rw_structured_buffer=new int[_buffers["particle_in_grid_particle_index_rw_structured_buffer"].count];
            _buffers["particle_in_grid_index_rw_structured_buffer"].GetData(particle_in_grid_index_rw_structured_buffer);
            _buffers["particle_in_grid_particle_index_rw_structured_buffer"].GetData(particle_in_grid_particle_index_rw_structured_buffer);
            
            Debug.LogFormat("排序的KV对数为{0}",particle_in_grid_index_rw_structured_buffer.Length);
            
            for(int i=0;i<particle_in_grid_index_rw_structured_buffer.Length;i++)
            {
                uint grid_index=(uint)particle_in_grid_index_rw_structured_buffer[i];
                Vector3Int  grid_index_3d=new Vector3Int((int)(grid_index%_gridResolution.x),(int)((grid_index/_gridResolution.x)%_gridResolution.y),(int)(grid_index/(_gridResolution.x*_gridResolution.y)));
                Assert.AreEqual(grid_index_3d.x+grid_index_3d.y*_gridResolution.x+grid_index_3d.z*_gridResolution.x*_gridResolution.y,particle_in_grid_index_rw_structured_buffer[i]);
                Debug.LogFormat("(K,V)对为{0},{1}",grid_index_3d,particle_in_grid_particle_index_rw_structured_buffer[i]);
            }
            

#endif
            
            //-----------------Force Calculation-----------------//
            ComputeShaderSetBuffer("ForceCalculationKernel","particle_position_rw_structured_buffer");
            ComputeShaderSetBuffer("ForceCalculationKernel","particle_velocity_rw_structured_buffer");
            ComputeShaderSetBuffer("ForceCalculationKernel","granule_data_rw_structured_buffer");
            ComputeShaderSetBuffer("ForceCalculationKernel","plane_data_rw_structured_buffer");
            ComputeShaderSetBuffer("ForceCalculationKernel","inertia_tensor_rw_structured_buffer");
            ComputeShaderSetBuffer("ForceCalculationKernel","particle_index_to_granule_index_rw_structured_buffer");
            ComputeShaderSetBuffer("ForceCalculationKernel","particle_initial_offset_rw_structured_buffer");
            ComputeShaderSetBuffer("ForceCalculationKernel","particle_contact_force_rw_structured_buffer");
            ComputeShaderSetBuffer("ForceCalculationKernel","particle_in_grid_index_rw_structured_buffer");
            ComputeShaderSetBuffer("ForceCalculationKernel","particle_in_grid_particle_index_rw_structured_buffer");
#if DEBUG_APPEND
            ComputeShaderSetBuffer("ForceCalculationKernel","debug_append_structured_buffer");
#endif
    
            cs.Dispatch(_kernels["ForceCalculationKernel"],Mathf.CeilToInt(_currentParticleCount/THREAD_GROUP_SIZE_X), 1, 1);
            
            //-----------------UpdateGranuleKernel-----------------//
            
            ComputeShaderSetBuffer("UpdateGranuleKernel","particle_position_rw_structured_buffer");
            ComputeShaderSetBuffer("UpdateGranuleKernel","particle_velocity_rw_structured_buffer");
            ComputeShaderSetBuffer("UpdateGranuleKernel","granule_data_rw_structured_buffer");
            ComputeShaderSetBuffer("UpdateGranuleKernel","plane_data_rw_structured_buffer");
            ComputeShaderSetBuffer("UpdateGranuleKernel","inertia_tensor_rw_structured_buffer");
            ComputeShaderSetBuffer("UpdateGranuleKernel","particle_index_to_granule_index_rw_structured_buffer");
            ComputeShaderSetBuffer("UpdateGranuleKernel","particle_initial_offset_rw_structured_buffer");
            ComputeShaderSetBuffer("UpdateGranuleKernel","particle_contact_force_rw_structured_buffer");
#if DEBUG_APPEND
            ComputeShaderSetBuffer("UpdateGranuleKernel","debug_append_structured_buffer");
#endif
            
            
            cs.Dispatch(_kernels["UpdateGranuleKernel"],Mathf.CeilToInt(_currentGranuleCount/THREAD_GROUP_SIZE_X), 1, 1);
            
            //-----------------UpdateParticleKernel-----------------//
            ComputeShaderSetBuffer("UpdateParticleKernel","particle_position_rw_structured_buffer");
            ComputeShaderSetBuffer("UpdateParticleKernel","particle_velocity_rw_structured_buffer");
            ComputeShaderSetBuffer("UpdateParticleKernel","granule_data_rw_structured_buffer");
            ComputeShaderSetBuffer("UpdateParticleKernel","particle_index_to_granule_index_rw_structured_buffer");
            ComputeShaderSetBuffer("UpdateParticleKernel","particle_initial_offset_rw_structured_buffer");
            ComputeShaderSetBuffer("UpdateParticleKernel","particle_in_grid_index_rw_structured_buffer");
            ComputeShaderSetBuffer("UpdateParticleKernel","particle_in_grid_particle_index_rw_structured_buffer");
            
            cs.Dispatch(_kernels["UpdateParticleKernel"],Mathf.CeilToInt(_currentParticleCount/THREAD_GROUP_SIZE_X), 1, 1);
 

            
#if DEBUG_APPEND
            

            // Vector3[] position = new Vector3[_buffers["particle_position_rw_structured_buffer"].count];
            // _buffers["particle_position_rw_structured_buffer"].GetData(position);
            //
            // for(int i=0;i<position.Length/2;i++)
            // {
            //     Debug.LogFormat("粒子{0}的位置为{1}",i,position[i+_bufferIndexBegin*_currentParticleCount]);
            // }

 
            
            ComputeBuffer.CopyCount(_buffers["debug_append_structured_buffer"],_buffers["debug_append_count_buffer"],0);
            int[] debug_append_count = new int[1];
            _buffers["debug_append_count_buffer"].GetData(debug_append_count);
            if(debug_append_count[0]>0)
            {
                Debug.LogFormat("debug buffer 中数据有{0}个",debug_append_count[0]);
                Debug.Log("debug buffer中的数据为:");
                Vector3[] debug_append_data=new Vector3[debug_append_count[0]];
                _buffers["debug_append_structured_buffer"].GetData(debug_append_data);
  
                for (int i = 0; i < debug_append_data.Length; i++)
                {
                    Debug.Log(debug_append_data[i]);
                }
                
                for(int i=0;i<debug_append_data.Length;i++)
                {
                    if (debug_append_data[i].z.Equals(111f))
                    {
                        bool flag = false;
                        for(int j=0;j<debug_append_data.Length;j++)
                        {
                            if(debug_append_data[j].z.Equals(222f)&&
                               debug_append_data[j].x.Equals(debug_append_data[i].x)&&
                               debug_append_data[j].y.Equals(debug_append_data[i].y))
                            {
                                flag = true;
                                break;
                            }
                        }
                        if(!flag)
                        {
                            Debug.LogFormat("未在邻域搜索中加载粒子{0}和粒子{1}的接触力",debug_append_data[i].x,debug_append_data[i].y);
                        }
                    }
                }
                
            }
            //清空Append Buffer
            _buffers["debug_append_structured_buffer"].SetCounterValue(0);
#endif
            
            _bufferIndexBegin=1-_bufferIndexBegin;


        }
        Matrix4x4 CalculateRefererenceInertiaTensor(List<Vector3> vertices)
        {
            Matrix4x4 I_ref = Matrix4x4.zero;
            float vertexMass = particleMass;
            for (int i = 0; i < vertices.Count; i++)
            {
                I_ref[0, 0] += vertexMass * vertices[i].sqrMagnitude;
                I_ref[1, 1] += vertexMass * vertices[i].sqrMagnitude;
                I_ref[2, 2] += vertexMass * vertices[i].sqrMagnitude;
                I_ref[0, 0] -= vertexMass * vertices[i][0] * vertices[i][0];
                I_ref[0, 1] -= vertexMass * vertices[i][0] * vertices[i][1];
                I_ref[0, 2] -= vertexMass * vertices[i][0] * vertices[i][2];
                I_ref[1, 0] -= vertexMass * vertices[i][1] * vertices[i][0];
                I_ref[1, 1] -= vertexMass * vertices[i][1] * vertices[i][1];
                I_ref[1, 2] -= vertexMass * vertices[i][1] * vertices[i][2];
                I_ref[2, 0] -= vertexMass * vertices[i][2] * vertices[i][0];
                I_ref[2, 1] -= vertexMass * vertices[i][2] * vertices[i][1];
                I_ref[2, 2] -= vertexMass * vertices[i][2] * vertices[i][2];
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

        private void SetupGrid()
        {

            Debug.LogFormat("grid resolution: {0},一共有{1}个Cell", _gridResolution,
                _gridResolution.x * _gridResolution.y * _gridResolution.z);


            sorter = new DeviceRadixSort(sortingShader, MaxParticleCount, ref temp0Buffer, ref temp1Buffer,
                ref temp2Buffer, ref temp3Buffer);
            
            

        }
    }

}

