using System.Collections.Generic;
using DataProcessor;
using UnityEngine;

public class TrainingArea : MonoBehaviour
{
    public string folderPath;
    public MannequinAgent mannequinAgentPrefab;

    private Rigidbody[] _rigidbody;

    private Dictionary<string, AnimationClip> AnimDict;

    public MotionData[] motions;

    public bool isManualMotion = false;
    public MotionData manualMotion;

    // Start is called before the first frame update
    private void Start()
    {
        // Time.fixedDeltaTime = 1 / 30f;
        InitArea();

#if UNITY_EDITOR
        int size = 1;
#else
        int size = 5;
#endif
        int gap = 20;
        for (var i = 0; i < size; ++i)
        for (var j = 0; j < size; ++j)
            Instantiate(mannequinAgentPrefab, new Vector3(
                gap / 2 * (1 - size + 2 * i),
                0,
                gap * -j
            ), Quaternion.identity, transform);
    }

    public void InitArea()
    {
        motions = Resources.LoadAll<MotionData>(folderPath);
    }

    public int GetRandomMotionIndex()
    {
        return Random.Range(0, motions.Length);
    }

    public MotionData GetMotion(int index)
    {
        if (isManualMotion && manualMotion != default)
            return manualMotion;

        return motions[index];
    }

    public int GetMotionTotalFrame(int index)
    {
        return motions[index].data.Count;
    }
}