using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace GraduationDesign
{
    public class RigidBodyRegister : MonoBehaviour
    {
        public static List<RegisterDataType> RigidBodyData = new List<RegisterDataType>();
        public struct RegisterDataType
        {
            public Vector3 Position;
            public Vector3 Velocity;
            public Vector3 AngularVelocity;
            public Quaternion Rotation;
            public List<Vector3> RigidBodiesParticleInitialOffset;
            public BodyType RigidBodyType;
        }
        public enum BodyType
        {
            Cube,
            Custom              //根据算法来自动生成表面的Particle
        }
        
        public BodyType bodyType = BodyType.Cube;  
        public float particle_radius = 0.1f;

        private void Awake()
        {
            switch (bodyType)
            {
                case BodyType.Cube:
                    RigidBodyData.Add(GenerateCube());
                    break;
            }
        }

        RegisterDataType GenerateCube()
        {
            //相当于是6个平面
            //Unity的Cube网格边长为1
            uint x_max_count = (uint)(transform.localScale.x / particle_radius / 2) + 1;
            uint y_max_count = (uint)(transform.localScale.y / particle_radius / 2) + 1;
            uint z_max_count = (uint)(transform.localScale.z / particle_radius / 2) + 1;
            
            List<Vector3> rigidBodiesParticeInitialOffset = new List<Vector3>();
            
            //只需要Cube表面的Particle
            for (uint i = 0; i < x_max_count; i++)
            {
                for (uint j = 0; j < y_max_count; j++)
                {
                    for (uint k = 0; k < z_max_count; k++)
                    {
                        if (i == 0 || i == x_max_count - 1 || j == 0 || j == y_max_count - 1 || k == 0 || k == z_max_count - 1)
                        {
                            Vector3 position = new Vector3(i * particle_radius * 2, j * particle_radius * 2, k * particle_radius * 2) - transform.localScale / 2;
                            rigidBodiesParticeInitialOffset.Add(position);  
                        }
                    }
                }
            }
            RegisterDataType data = new RegisterDataType();
            data.Position = transform.position;
            data.Velocity = Vector3.zero;
            data.AngularVelocity = Vector3.zero;
            data.Rotation = transform.rotation;
            data.RigidBodiesParticleInitialOffset = rigidBodiesParticeInitialOffset;
            data.RigidBodyType = BodyType.Cube;
            return data;
        }


    }

}
