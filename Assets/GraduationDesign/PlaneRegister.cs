using System;
using System.Collections;
using System.Collections.Generic;
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
            Vector3 origin_in_unity=transform.position;
            Vector3 normal_in_unity=transform.up;
            Vector3 a_axis_in_unity=transform.right;
            Vector3 b_axis_in_unity=transform.forward;
            Vector3 origin=origin_in_unity-a_axis_in_unity*transform.localScale.x/2-b_axis_in_unity*transform.localScale.z/2;
            Vector3 normal=normal_in_unity;
            Vector3 a_axis=a_axis_in_unity*transform.localScale.x;
            Vector3 b_axis=b_axis_in_unity*transform.localScale.z;
            RegisterPlane(origin, normal, a_axis, b_axis, particle_radius);
        }
    }
}

