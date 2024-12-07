using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneRegisterManager : MonoBehaviour
{
    private static PlaneRegisterManager _instance;
    public static PlaneRegisterManager Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindObjectOfType<PlaneRegisterManager>();
            }
            return _instance;
        }
    }
    
    public ComputeBuffer _planeBuffer;   //粒子数据
    
    public struct PlaneData
    {
        public Vector3 position;//平面上的点
        public Vector3 normal;//平面的法线
    }
    
    public List<PlaneData> planeDatas = new List<PlaneData>();
    
    private void Start()
    {
        _planeBuffer = new ComputeBuffer(planeDatas.Count, sizeof(float) * 6);
        _planeBuffer.SetData(planeDatas);
        for(int i = 0; i < planeDatas.Count; i++)
        {
            Debug.Log("PlaneData: " + planeDatas[i].position + " " + planeDatas[i].normal);
        }
    }
    
    private void OnDestroy()
    {
        _planeBuffer.Release();
    }
    
    
}
