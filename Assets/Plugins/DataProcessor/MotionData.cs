using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;

#endif // UNITY_EDITOR

public class MotionData : ScriptableObject
{
    public string characterName;
    public string motionName;
    public int totalFrame;
    public float fps = 60;
    public List<SkeletonData> data;

    public void Init(int frameCount)
    {
        totalFrame = frameCount;
        data = new List<SkeletonData>(frameCount);
    }

    public void Save()
    {
#if UNITY_EDITOR
        var path = $"Assets/output/{motionName}.motion.asset";
        Debug.Log($"Saving to {path}");
        if (!AssetDatabase.IsValidFolder("Assets/output"))
            AssetDatabase.CreateFolder("Assets", "output");
        AssetDatabase.CreateAsset(this, path);
        AssetDatabase.SaveAssets();
#endif // UNITY_EDITOR
    }
}