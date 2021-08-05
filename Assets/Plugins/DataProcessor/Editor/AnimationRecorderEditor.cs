using DataProcessor;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(AnimationRecorder))]
public class AnimationRecorderEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        if (GUILayout.Button("Export"))
            (target as AnimationRecorder)?.LoadFolder();
    }
}