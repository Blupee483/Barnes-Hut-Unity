using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Display : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Sim2D sim;

    [Header("Rendering stuff")]
    [SerializeField] private Mesh mesh;
    [SerializeField] private Material mat;
    private Matrix4x4[] matrices;
    private RenderParams rp;

    private uint numBodies;
    private Body[] bodies;

    void Start()
    {
        //initialize
        numBodies = sim.numBodies;
        rp = new RenderParams(mat);
        matrices = new Matrix4x4[numBodies];
    }


    void LateUpdate()
    {
        //populate the transformation matrices for each body
        numBodies = (uint)sim.bodies.Length;
        bodies = sim.bodies;
        Vector3 scale;
        Quaternion identityRot = Quaternion.identity;
        for(int i = 0; i < numBodies; i++)
        {
            scale = new Vector3(bodies[i].radius*2, bodies[i].radius*2, 1f);
            matrices[i].SetTRS(bodies[i].position, identityRot, scale);
        }

        //render using unity's api in batches of 1023
        int remaining = (int)numBodies;
        int offset = 0;
        while(remaining > 0)
        {
            int countToRender = Mathf.Min(remaining, 1023);

            Graphics.RenderMeshInstanced(rp, mesh, 0, matrices, countToRender, offset);

            offset += countToRender;
            remaining -= countToRender;
        }
    }
}

