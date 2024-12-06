using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ParticleRegister : MonoBehaviour
{
    private MeshFilter meshFilter;

    List<ParticelRegisterManager.RigidBodyParticleData> rigidBodyParticleDatas = new List<ParticelRegisterManager.RigidBodyParticleData>();
    private void Awake()
    {
        meshFilter = GetComponent<MeshFilter>();
        CreatePlaneParticles();
        RegisterData();
    }

    void CreatePlaneParticles()
    {
        //TEST，这里为了方便测试，在项目中使用的是Plane
        const float particleRadius = 0.1f;
        //从[-3,3]*[-3,3]生成粒子
        for (float x = -3; x < 3; x += particleRadius*2)
        {
            for (float z = -3; z < 3; z += particleRadius*2)
            {
                ParticelRegisterManager.RigidBodyParticleData data = new ParticelRegisterManager.RigidBodyParticleData();
                data.position = new Vector3(x, 0, z);
                rigidBodyParticleDatas.Add(data);
            }
        }
    }

    void RegisterData()
    {
        ParticelRegisterManager.Instance.rigidBodyParticleDatas.AddRange(rigidBodyParticleDatas);
    }
}
