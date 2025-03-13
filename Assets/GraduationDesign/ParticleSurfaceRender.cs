using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ParticleSurfaceRender : MonoBehaviour
{
    public Material material;
    public Mesh mesh;
    
    private GraphicsBuffer commandBuf;
    private GraphicsBuffer.IndirectDrawIndexedArgs[] commandData;
    private RenderParams rp;
    private ComputeBuffer particelPositionBuffer;
    private float particle_radius = 0.1f;
    private List<Vector3> positions;
    
    void RenderParticle()
    {
        material.SetBuffer("_PositionBuffer", particelPositionBuffer);
        material.SetFloat("_ParticleRadius", particle_radius);
        //设置transform矩阵
        material.SetMatrix("_ObjectToWorld", transform.localToWorldMatrix);
        Graphics.RenderMeshIndirect(rp, mesh, commandBuf, 1);
    }

    public void SetupData(List<Vector3> positions,float particle_radius)
    {
        this.positions = positions;
        this.particle_radius = particle_radius;
        commandBuf = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments, 1, GraphicsBuffer.IndirectDrawIndexedArgs.size);
        commandData = new GraphicsBuffer.IndirectDrawIndexedArgs[1];
        rp = new RenderParams(material);
        rp.worldBounds = new Bounds(Vector3.zero, 100*Vector3.one); // use tighter bounds for better FOV culling
        commandData[0].indexCountPerInstance = mesh.GetIndexCount(0);
        commandData[0].instanceCount = (uint)positions.Count;
        commandBuf.SetData(commandData);
        particelPositionBuffer = new ComputeBuffer(positions.Count, sizeof(float) * 3);
        particelPositionBuffer.SetData(positions);
    }

    private void Update()
    {
        RenderParticle();
    }
}
