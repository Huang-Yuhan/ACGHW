using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace GraduationDesign
{
    [CustomEditor(typeof(RigidBodyRegister))]
    public class RigidBodyRegisterEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();
            RigidBodyRegister rigidBodyRegister = (RigidBodyRegister)target;
            if (GUILayout.Button("Generate"))
            {
                rigidBodyRegister.Generate();
            }
        }
    }
}