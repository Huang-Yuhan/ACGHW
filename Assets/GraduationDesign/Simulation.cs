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
        
        //-----------------Compute Buffer-----------------//
        private ComputeBuffer _particlePositionBuffer;      //粒子位置
        private ComputeBuffer _particleVelocityBuffer;      //粒子速度
        
        //-----------------Kernels-----------------//
        Dictionary<string,int> _kernels = new Dictionary<string, int>();
        
        //-----------------Compute Buffers-----------------//
        
        Dictionary<string,int> _bufferIds = new Dictionary<string, int>();
        Dictionary<string,ComputeBuffer> _buffers = new Dictionary<string, ComputeBuffer>();
        
        
        
    }

}

