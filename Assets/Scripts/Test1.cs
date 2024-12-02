using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Object = System.Object;

public class Test1 : MonoBehaviour
{
    public Material material;
    public Mesh mesh;

    GraphicsBuffer commandBuf;
    GraphicsBuffer.IndirectDrawIndexedArgs[] commandData;
    
    private RenderParams rp;
    
    public ComputeShader computeShader;
    private int _kernel;
    private ComputeBuffer _buffer;

    struct PositionData
    {
        public Vector3 position;
        public Vector4 color;
    }
    
    void Start()
    {
        commandBuf = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments, 1, GraphicsBuffer.IndirectDrawIndexedArgs.size);
        commandData = new GraphicsBuffer.IndirectDrawIndexedArgs[1];

        rp = new RenderParams(material);
        rp.worldBounds = new Bounds(Vector3.zero, 100*Vector3.one); // use tighter bounds for better FOV culling
        commandData[0].indexCountPerInstance = mesh.GetIndexCount(0);
        commandData[0].instanceCount = 256*256;
        commandBuf.SetData(commandData);
        
        _kernel = computeShader.FindKernel("CSMain");
        PositionData[] data = new PositionData[256*256];
        for (int i = 0; i < 256; i++)
        {
            for (int j = 0; j < 256; j++)
            {
                data[i * 256 + j].position = new Vector3(i, 0, j);
                data[i * 256 + j].color = new Vector4(1, 0, 0, 1);
            }
        }
        _buffer = new ComputeBuffer(256*256, sizeof(float)*7);
        _buffer.SetData(data);
    }

    void OnDestroy()
    {
        commandBuf?.Release();
        commandBuf = null;
        _buffer?.Release();
        _buffer = null;
    }

    void Update()
    {
        material.SetBuffer("_PositionBuffer", _buffer);
        computeShader.SetBuffer(_kernel, "_PositionBuffer", _buffer);
        computeShader.Dispatch(_kernel, 256/8, 256/8, 1);
        Graphics.RenderMeshIndirect(rp, mesh, commandBuf, 1);
    }
}
