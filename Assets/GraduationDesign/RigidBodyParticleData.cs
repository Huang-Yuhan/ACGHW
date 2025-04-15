using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using UnityEditor;


namespace GraduationDesign
{
    [CreateAssetMenu(fileName = "RigidBodyParticleData", menuName = "GraduationDesign/RigidBodyParticleData")]
    public class RigidBodyParticleData : ScriptableObject
    {
        public Vector3 Position;
        public Vector3 Velocity;
        public Vector3 AngularVelocity;
        public Quaternion Rotation;
        public List<Vector3> RigidBodiesParticleInitialOffset;
        public float ParticleRadius;
        public float ParticleMass;
    }
}
