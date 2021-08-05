using DataProcessor;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(AnimationPlayer))]
public class AnimationPlayerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        if (GUILayout.Button("Play"))
            (target as AnimationPlayer)?.PlayCoroutine();
    }
}