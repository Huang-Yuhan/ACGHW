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
            public Vector3 position;
            public Vector3 normal;

            public static int GetSize()
            {
                return sizeof(float) * 6;
            }
        }
        
        public static List<plane_data_type> plane_data = new List<plane_data_type>();
        
        public static void RegisterPlane(Vector3 position, Vector3 normal)
        {
            plane_data_type plane_data_temp = new plane_data_type();
            plane_data_temp.position = position;
            plane_data_temp.normal = normal;
            plane_data.Add(plane_data_temp);
        }

        private void Awake()
        {
            RegisterPlane(transform.position, transform.up);
        }
    }
}

