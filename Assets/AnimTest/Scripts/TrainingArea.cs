using System.Collections.Generic;
using System.Linq;
using DataProcessor;
using UnityEngine;
using UnityEngine.Assertions;

public class TrainingArea : MonoBehaviour
{
    public MannequinAgent mannequinAgentPrefab;
    public GameObject mannequinRef;

    private Rigidbody[] _rigidbody;

    public readonly Dictionary<string, MotionData> MotionDict = new Dictionary<string, MotionData>();
    private Dictionary<string, AnimationClip> AnimDict;

    // Start is called before the first frame update
    private void Start()
    {
        InitArea();

#if UNITY_EDITOR
        int size = 1;
#else
        int size = 5;
#endif
        int gap = 4;
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
        GetAllMotionTXT("Motions");
        GetAllAnimClips("MannequinAnimation");
    }

    private void GetAllMotionTXT(string dirPath)
    {
        var infos = Resources.LoadAll<TextAsset>(dirPath);
        foreach (var textAsset in infos)
        {
            if (textAsset == default) continue;

            var motionData = AnimationImporter.Import(textAsset);
            MotionDict[textAsset.name] = motionData;
        }
    }

    private void GetAllAnimClips(string dirPath)
    {
        AnimDict = Resources.LoadAll<AnimationClip>(dirPath)
            .Where(p => MotionDict.ContainsKey(p.name))
            .ToDictionary(p => p.name);

        Assert.IsTrue(AnimDict.Count == MotionDict.Count,
            $"{MotionDict.Count - AnimDict.Count} Animation Clips are lost");
    }

    public KeyValuePair<string, MotionData> GetRandomMotion()
    {
        var idx = Random.Range(0, MotionDict.Count);
        return MotionDict.ElementAt(idx);
    }

    public AnimationClip GetAnimationClip(string key)
    {
        return AnimDict[key];
    }
}