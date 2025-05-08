using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JustRotate : MonoBehaviour
{
    public float rotationSpeed = 10f;
    private void Update()
    {
        //改变物体的旋转只改变Z
        Vector3 euler = transform.rotation.eulerAngles;
        euler.z += rotationSpeed * Time.deltaTime;
        transform.rotation = Quaternion.Euler(euler);
    }
}
