using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

namespace GraduationDesign
{
    public class PlaneRegister : MonoBehaviour
    {
        public struct plane_data_type
        {
            public Vector3 normal;
            public Vector3 position;
            public Vector3 a_axis;
            public Vector3 b_axis;
            public float particle_radius;


            public static int GetSize()
            {
                return sizeof(float) * 3 * 4 + sizeof(float);
            }
        }
        
        public static List<plane_data_type> plane_data = new List<plane_data_type>();
        public float particle_radius = 0.1f;
        public bool isRenderParticle = false;
        public Material material;
        public Mesh mesh;
        private GraphicsBuffer commandBuf;
        private GraphicsBuffer.IndirectDrawIndexedArgs[] commandData;
        private RenderParams rp;
        private ComputeBuffer particelPositionBuffer;
        private plane_data_type thisOneplaneData;
        public static void RegisterPlane(Vector3 position, Vector3 normal, Vector3 a_axis, Vector3 b_axis, float particle_radius)
        {
            plane_data_type plane_data_temp = new plane_data_type();
            plane_data_temp.position = position;
            plane_data_temp.normal = normal;
            plane_data_temp.a_axis = a_axis;
            plane_data_temp.b_axis = b_axis;
            plane_data_temp.particle_radius = particle_radius;
            plane_data.Add(plane_data_temp);
            
            Debug.LogFormat("Register Plane: {0}, {1}, {2}, {3}, {4}", position, normal, a_axis, b_axis, particle_radius);
        }


        private void Awake()
        {
            
            //找到平面网格中的四个点
            float x_min = float.MaxValue;
            float x_max = float.MinValue;
            float z_min = float.MaxValue;
            float z_max = float.MinValue;
            
            MeshFilter meshFilter = GetComponent<MeshFilter>();
            Mesh mesh = meshFilter.mesh;
            Vector3[] vertices = mesh.vertices;
            foreach (Vector3 vertex in vertices)
            {
                if (vertex.x < x_min)
                {
                    x_min = vertex.x;
                }
                if (vertex.x > x_max)
                {
                    x_max = vertex.x;
                }
                if (vertex.z < z_min)
                {
                    z_min = vertex.z;
                }
                if (vertex.z > z_max)
                {
                    z_max = vertex.z;
                }
            }
            
            Vector3 origin_in_unity=transform.position;
            Vector3 normal_in_unity=transform.up;
            Vector3 a_axis_in_unity=transform.right;
            Vector3 b_axis_in_unity=transform.forward;
            float x_length_in_unity=(x_max-x_min)*transform.localScale.x;
            float z_length_in_unity=(z_max-z_min)*transform.localScale.z;
            Vector3 origin=origin_in_unity-a_axis_in_unity*x_length_in_unity/2-b_axis_in_unity*z_length_in_unity/2;
            Vector3 normal=normal_in_unity;
            Vector3 a_axis=a_axis_in_unity*x_length_in_unity;
            Vector3 b_axis=b_axis_in_unity*z_length_in_unity;
            RegisterPlane(origin, normal, a_axis, b_axis, particle_radius);
            
            thisOneplaneData = plane_data[plane_data.Count - 1];
            
        }

        void RenderParticle()
        {
            material.SetBuffer("_PositionBuffer", particelPositionBuffer);
            material.SetFloat("_ParticleRadius", particle_radius);
            Graphics.RenderMeshIndirect(rp, mesh, commandBuf, 1);
        }


        
        private void Start()
        {
            uint x_max_count = (uint)(thisOneplaneData.a_axis.magnitude / particle_radius/2)+1;
            uint z_max_count = (uint)(thisOneplaneData.b_axis.magnitude / particle_radius/2)+1;
            Debug.LogFormat("x_max_count: {0}, z_max_count: {1}", x_max_count, z_max_count);
            commandBuf = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments, 1, GraphicsBuffer.IndirectDrawIndexedArgs.size);
            commandData = new GraphicsBuffer.IndirectDrawIndexedArgs[1];
            rp = new RenderParams(material);
            rp.worldBounds = new Bounds(Vector3.zero, 100*Vector3.one); // use tighter bounds for better FOV culling
            commandData[0].indexCountPerInstance = mesh.GetIndexCount(0);
            commandData[0].instanceCount = x_max_count*z_max_count;
            commandBuf.SetData(commandData);
            particelPositionBuffer = new ComputeBuffer((int)(x_max_count * z_max_count), sizeof(float) * 3);
            Vector3[] data = new Vector3[x_max_count*z_max_count];
            for (int i = 0; i < x_max_count; i++)
            {
                for (int j = 0; j < z_max_count; j++)
                {
                    data[i * z_max_count + j] = thisOneplaneData.position +
                                                thisOneplaneData.a_axis.normalized * i * particle_radius*2 +
                                                thisOneplaneData.b_axis.normalized * j * particle_radius*2;
                }
            }
            particelPositionBuffer.SetData(data);
            
        }

        private void Update()
        {
            if (isRenderParticle)
            {
                GetComponent<MeshRenderer>().enabled = false;
                RenderParticle();
            }else
            {
                GetComponent<MeshRenderer>().enabled = true;
            }
        }
        
        void OnDestroy()
        {
            commandBuf?.Release();
            commandBuf = null;
            particelPositionBuffer?.Release();
            particelPositionBuffer = null;
        }

        private void OnDrawGizmosSelected()
        {
            //绘制原点
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(thisOneplaneData.position, 0.1f);
            //绘制两个轴
            Gizmos.color = Color.green;
            Gizmos.DrawLine(thisOneplaneData.position, thisOneplaneData.position + thisOneplaneData.a_axis);
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(thisOneplaneData.position, thisOneplaneData.position + thisOneplaneData.b_axis);
        }
    }
}

