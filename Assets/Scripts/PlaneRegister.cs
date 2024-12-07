using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneRegister : MonoBehaviour
{
    private MeshFilter _meshFilter;

    private void Awake()
    {
        _meshFilter = GetComponent<MeshFilter>();
        PlaneRegisterManager.Instance.planeDatas.Add(new PlaneRegisterManager.PlaneData()
        {
            position = transform.position,
            normal = transform.up
        });
    }
    

    
    
    
}
