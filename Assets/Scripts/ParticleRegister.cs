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
        float lower_bound = -5;
        float upper_bound = 5;
        
        for (float x = lower_bound; x < upper_bound; x += particleRadius*2)
        {
            for (float z = lower_bound; z < upper_bound; z += particleRadius*2)
            {
                ParticelRegisterManager.RigidBodyParticleData data = new ParticelRegisterManager.RigidBodyParticleData();
                data.position = transform.TransformPoint(new Vector3(x, 0, z));
                rigidBodyParticleDatas.Add(data);
            }
        }
    }

    void RegisterData()
    {
        ParticelRegisterManager.Instance.rigidBodyParticleDatas.AddRange(rigidBodyParticleDatas);
    }
}
