using System.Collections.Generic;
using UnityEngine;

public class TrainingArea : MonoBehaviour
{
    public MannequinAgent mannequinAgentPrefab;

    private Rigidbody[] _rigidbody;

    private Dictionary<string, AnimationClip> AnimDict;

    public MotionData[] motions;

    // Start is called before the first frame update
    private void Start()
    {
        InitArea();

#if UNITY_EDITOR
        int size = 1;
#else
        int size = 5;
#endif
        int gap = 5;
        for (var i = 0; i < size; ++i)
        for (var j = 0; j < size; ++j)
            Instantiate(mannequinAgentPrefab, new Vector3(
                gap / 2 * (1 - size + 2 * i),
                0,
                gap * (-1 - j)
            ), Quaternion.identity, transform);
    }

    public void InitArea()
    {
        motions = Resources.LoadAll<MotionData>("Motions");
    }

    public int GetRandomMotionIndex()
    {
        return Random.Range(0, motions.Length);
    }

    public MotionData GetMotion(int index)
    {
        return motions[index];
    }

    public int GetMotionTotalFrame(int index)
    {
        return motions[index].data.Count;
    }
}