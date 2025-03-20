using System;
using System.Collections;
using System.Collections.Generic;
using GPUSorting.Runtime;
using UnityEngine;
using Random = UnityEngine.Random;

public class GPUSort : MonoBehaviour
{
    public ComputeShader sortShader;
    public int numElements = 16;
    private DeviceRadixSort radixSort;
    private ComputeBuffer temp0buffer;
    ComputeBuffer temp1buffer;
    ComputeBuffer temp2buffer;

    private void Start()
    {
        int[] data = new int[numElements];
        for (int i = 0; i < numElements; i++)
        {
            data[i] = Random.Range(int.MinValue, int.MaxValue);
            Debug.Log(data[i]);
        }

        Debug.Log("ready to sort");
        
        radixSort = new DeviceRadixSort(sortShader, numElements,ref  temp0buffer, ref temp1buffer, ref temp2buffer);
        
        ComputeBuffer dataBuffer = new ComputeBuffer(numElements, sizeof(int));
        dataBuffer.SetData(data);

        radixSort.Sort(
            numElements,
            dataBuffer,
            temp0buffer,
            temp1buffer,
            temp2buffer,
            typeof(int),
            true
            );
        
        dataBuffer.GetData(data);
        dataBuffer.Release();
        
        for (int i = 0; i < numElements; i++)
        {
            Debug.Log(data[i]);
        }
        
        temp0buffer.Release();
        temp1buffer.Release();
        temp2buffer.Release();

    }
}
