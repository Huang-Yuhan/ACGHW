using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace GraduationDesign
{
    public class RigidBodyRegister : MonoBehaviour
    {
        public static List<RigidBodyDataType> RigidBodyData = new List<RigidBodyDataType>();

        public static int ParticleSum
        {
            get
            {
                int sum = 0;
                for(int i=0;i<RigidBodyData.Count;i++)
                {
                    sum += RigidBodyData[i].data.RigidBodiesParticleInitialOffset.Count;
                }
                return sum;
            }
        }
        
        public struct RigidBodyDataType
        {
            public RigidBodyParticleData data;
            public GameObject gameObject;
        }
        
        public float particle_radius = 0.1f;
        public float mass;
        public RigidBodyParticleData rigidBodyParticleData;
        public bool isControlledBySimulation = false;
        public bool isDisplayParticle = false;
        public float lowerBound = 0.1f;

        public void Generate()
        {
            rigidBodyParticleData = GenerateCustom();
            //保存到资源
            string path = "Assets/GraduationDesign/ScriptObject/RigidBodyParticleData/" + gameObject.name + ".asset";
            //如果已经存在，则删除
            if (System.IO.File.Exists(path))
            {
                AssetDatabase.DeleteAsset(path);
            }
            //创建新的资源
            AssetDatabase.CreateAsset(rigidBodyParticleData, path);
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            //RigidBodyData.Add(registerData);
        }

        private void Awake()
        {
            if (rigidBodyParticleData == null)
            {
                Debug.LogWarning("No rigid body type selected");
                return;
            }
            var registerData = new RigidBodyDataType();
            registerData.data = rigidBodyParticleData;
            registerData.gameObject = gameObject;
            RigidBodyData.Add(registerData);
        }

        // RegisterDataType GenerateCube()
        // {
        //     //相当于是6个平面
        //     //Unity的Cube网格边长为1
        //     var voxelSize = particle_radius * 2;
        //     // uint x_max_count = (uint)(transform.localScale.x / particle_radius / 2) + 1;
        //     // uint y_max_count = (uint)(transform.localScale.y / particle_radius / 2) + 1;
        //     // uint z_max_count = (uint)(transform.localScale.z / particle_radius / 2) + 1;
        //     int x_max_count = Mathf.CeilToInt(transform.localScale.x / voxelSize);
        //     int y_max_count = Mathf.CeilToInt(transform.localScale.y / voxelSize);
        //     int z_max_count = Mathf.CeilToInt(transform.localScale.z / voxelSize);
        //     
        //     List<Vector3> rigidBodiesParticeInitialOffset = new List<Vector3>();
        //     
        //     //只需要Cube表面的Particle
        //     for (uint i = 0; i <= x_max_count; i++)
        //     {
        //         for (uint j = 0; j <= y_max_count; j++)
        //         {
        //             for (uint k = 0; k <= z_max_count; k++)
        //             {
        //                 if (i == 0 || i == x_max_count  || j == 0 || j == y_max_count  || k == 0 || k == z_max_count )
        //                 {
        //                     Vector3 position = new Vector3(i * particle_radius * 2, j * particle_radius * 2, k * particle_radius * 2) - transform.localScale / 2;
        //                     rigidBodiesParticeInitialOffset.Add(position);  
        //                     //Debug.LogFormat("{0}",position);
        //                 }
        //             }
        //         }
        //     }
        //     RegisterDataType data = new RegisterDataType();
        //     data.Position = transform.position;
        //     data.Velocity = Vector3.zero;
        //     data.AngularVelocity = Vector3.zero;
        //     data.Rotation = transform.rotation;
        //     data.RigidBodiesParticleInitialOffset = rigidBodiesParticeInitialOffset;
        //     data.ParticleRadius = particle_radius;
        //     data.ParticleMass = mass;
        //     Debug.LogFormat("Cube Particle Count: {0}", rigidBodiesParticeInitialOffset.Count);
        //     return data;
        // }

        RigidBodyParticleData GenerateCustom()
        {
            RigidBodyParticleData data = ScriptableObject.CreateInstance<RigidBodyParticleData>();
            var mesh= GetComponent<MeshFilter>().sharedMesh;
            var vertices = mesh.vertices.Clone() as Vector3[];
            var triangles = mesh.triangles;
            var boundingBoxXMin=float.MaxValue;
            var boundingBoxXMax=float.MinValue;
            var boundingBoxYMin=float.MaxValue;
            var boundingBoxYMax=float.MinValue;
            var boundingBoxZMin=float.MaxValue;
            var boundingBoxZMax=float.MinValue;
            
            for (int i = 0; i < vertices.Length; i++)
            {
                vertices[i]=transform.TransformPoint(vertices[i]);
                var vertex = vertices[i];
                boundingBoxXMin = Mathf.Min(vertex.x, boundingBoxXMin);
                boundingBoxXMax = Mathf.Max(vertex.x, boundingBoxXMax);
                boundingBoxYMin = Mathf.Min(vertex.y, boundingBoxYMin);
                boundingBoxYMax = Mathf.Max(vertex.y, boundingBoxYMax);
                boundingBoxZMin = Mathf.Min(vertex.z, boundingBoxZMin);
                boundingBoxZMax = Mathf.Max(vertex.z, boundingBoxZMax);
            }
            
            float voxelSize = particle_radius * 2;
            int xCount = Mathf.CeilToInt((boundingBoxXMax - boundingBoxXMin) / voxelSize);
            int yCount = Mathf.CeilToInt((boundingBoxYMax - boundingBoxYMin) / voxelSize);
            int zCount = Mathf.CeilToInt((boundingBoxZMax - boundingBoxZMin) / voxelSize);
            Vector3 origin = new Vector3(boundingBoxXMin, boundingBoxYMin, boundingBoxZMin);

            var samples = getSphereSample(576);

            var computeShader = AssetDatabase.LoadAssetAtPath<ComputeShader>("Assets/GraduationDesign/SDFCalculation.compute");
            
            ComputeBuffer triangleBuffer = new ComputeBuffer(triangles.Length, sizeof(int));
            triangleBuffer.SetData(triangles);
            ComputeBuffer vertexBuffer = new ComputeBuffer(vertices.Length, sizeof(float) * 3);
            vertexBuffer.SetData(vertices);
            ComputeBuffer sampleBuffer = new ComputeBuffer(samples.Length, sizeof(float) * 3);
            sampleBuffer.SetData(samples);
            ComputeBuffer resultBuffer = new ComputeBuffer((int)1e6, sizeof(float) * 3, ComputeBufferType.Append);
            resultBuffer.SetCounterValue(0);
            var kenel = computeShader.FindKernel("CSMain");
            computeShader.SetBuffer(kenel, "Triangles", triangleBuffer);
            computeShader.SetBuffer(kenel, "Vertices", vertexBuffer);
            computeShader.SetBuffer(kenel, "Samples", sampleBuffer);
            computeShader.SetBuffer(kenel, "Result", resultBuffer);
            computeShader.SetInt("SampleCount", samples.Length);
            computeShader.SetInt("TriangleCount", triangles.Length / 3);
            computeShader.SetFloat("VoxelSize", voxelSize);
            computeShader.SetVector("Origin", origin);
            computeShader.SetInt("XCount", xCount);
            computeShader.SetInt("YCount", yCount);
            computeShader.SetInt("ZCount", zCount);
            computeShader.SetFloat("lowerBound", lowerBound);
            computeShader.Dispatch(0, xCount / 8 + 1, yCount / 8 + 1, zCount / 8 + 1);
            
            Debug.Log("ComputeShader Dispatch Done");
            //计算完毕，获取结果
            ComputeBuffer.CopyCount(resultBuffer, triangleBuffer, 0);
            int[] count = new int[1];
            triangleBuffer.GetData(count);
            int particleCount = count[0];
            Vector3[] result = new Vector3[particleCount];
            resultBuffer.GetData(result);
            resultBuffer.Dispose();
            triangleBuffer.Dispose();
            vertexBuffer.Dispose();
            sampleBuffer.Dispose();

            //现在的result是世界坐标系下的
            //需要转换为offset
            for (int i = 0; i < result.Length; i++)
            {
                //不考虑Scale，但是要考虑旋转
                result[i] = Quaternion.Inverse(transform.rotation) * (result[i] - transform.position);
            }
            
            data.Position = transform.position;
            data.Velocity = Vector3.zero;
            data.AngularVelocity = Vector3.zero;
            data.Rotation = transform.rotation;
            data.RigidBodiesParticleInitialOffset = new List<Vector3>(result);
            data.ParticleRadius = particle_radius;
            data.ParticleMass = mass;
            data.isControlledBySimulation = isControlledBySimulation? 1u : 0u;
            Debug.LogFormat("Custom Particle Count: {0}", result.Length);
            
            return data;
        }
        
        Vector3[] getSphereSample(int sampleCount)
        {
            Vector3[] samples = new Vector3[sampleCount];
            for (int i = 0; i < sampleCount; i++)
            {
                samples[i]=UnityEngine.Random.onUnitSphere;
            }
            return samples;
        }

        

        void OnDrawGizmos()
        {
            if (!isDisplayParticle|| rigidBodyParticleData == null)
                return;
            //显示表面的Particle
            var offsets = rigidBodyParticleData.RigidBodiesParticleInitialOffset;
            if(offsets == null)
                return;
            for (int i = 0; i < offsets.Count; i++)
            {
                Gizmos.color = Color.red;
                var pos = transform.position + Quaternion.Euler(rigidBodyParticleData.Rotation.eulerAngles) * offsets[i];
                Gizmos.DrawSphere(pos, particle_radius);
            }

            
        }


    }

}
