using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(Transform))]
public class TransformEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        EditorGUILayout.Vector3Field("World Rot",((Transform) target).rotation.eulerAngles);
        EditorGUILayout.Vector3Field("Local Rot",((Transform) target).localRotation.eulerAngles);
        var AB = ((Transform) target).GetComponent<ArticulationBody>();
        if (AB == default) return;
        if(AB.jointPosition.dofCount == 3)
            EditorGUILayout.Vector3Field("Anchor Rot",new Vector3(AB.jointPosition[0], AB.jointPosition[1], AB.jointPosition[2]) / Mathf.PI * 180);
        else if(AB.jointPosition.dofCount == 1)
            EditorGUILayout.Vector3Field("Anchor Rot",new Vector3(0, 0, AB.jointPosition[0]) / Mathf.PI * 180);
    }
}
