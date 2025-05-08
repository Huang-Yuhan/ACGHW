using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LastTransform : MonoBehaviour
{
    public Vector3 lastPosition;
    public Quaternion lastRotation;

    public Vector3 lastUpdateVelocity
    {
        get
        {
            if (lastPosition != null)
            {
                return (transform.position - lastPosition) / Time.deltaTime;
            }
            else
            {
                return Vector3.zero;
            }
        }
    }
    
    public Vector3 lastUpdateAngularVelocity
    {
        get
        {
            if (lastRotation != null)
            {
                return Quaternion.AngleAxis(Quaternion.Angle(lastRotation, transform.rotation), transform.up) * (transform.rotation.eulerAngles - lastRotation.eulerAngles) / Time.deltaTime;
            }
            else
            {
                return Vector3.zero;
            }
        }
    }

    IEnumerator updateCoroutine;



    private void Awake()
    {
        lastPosition = transform.position;
        lastRotation = transform.rotation;
        updateCoroutine = UpdateCoroutine();
        StartCoroutine(updateCoroutine);
    }

    IEnumerator UpdateCoroutine()
    {
        while (true)
        {
            yield return new WaitForFixedUpdate();
            // Debug.LogFormat("LastTransform: {0}", lastPosition);
            // Debug.LogFormat("NowTransform: {0}", transform.position);
            lastPosition = transform.position;
            lastRotation = transform.rotation;

        }
    }

 
}
