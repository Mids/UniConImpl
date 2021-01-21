using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class TransformData
{
    public Vector3 position;
    public Quaternion rotation;
    public Vector3 localPosition;
    public Quaternion localRotation;
    public Vector3 velocity;
    public Vector3 angularVelocity;

    public TransformData(Transform t)
    {
        position = t.position;
        rotation = t.rotation;
        localPosition = t.localPosition;
        localRotation = t.localRotation;
    }
}

public class TrainingArea : MonoBehaviour
{
    public MannequinAgent mannequinAgentPrefab;
    public GameObject mannequinRef;
    public Animator _animatorRef;
    public float animationLength;

    private static readonly int Reset = Animator.StringToHash("Reset");

    public List<Dictionary<RagdollJoint, TransformData>> RefList = new List<Dictionary<RagdollJoint, TransformData>>();

    private float passedTime = 0f;
    public const float deltaTime = 0.008f;


    // Start is called before the first frame update
    private IEnumerator Start()
    {
        _animatorRef = mannequinRef.GetComponentInChildren<Animator>();
        _animatorRef.speed = 0f;
        animationLength = _animatorRef.GetCurrentAnimatorClipInfo(0)[0].clip.length;

        var refTransforms = _animatorRef.GetComponentsInChildren<Transform>();


        while (passedTime < animationLength)
        {
            _animatorRef.Play("Idle", 0, passedTime / animationLength);

            yield return null;

            var newDict = new Dictionary<RagdollJoint, TransformData>();

            foreach (var kvp in RagdollData.RagdollJointNames)
            {
                var t = refTransforms.FirstOrDefault(p => p.name.Contains(kvp.Value));
                if (t == default) continue;

                newDict[kvp.Key] = new TransformData(t);
            }

            RefList.Add(newDict);

            passedTime += deltaTime;
        }

        for (var index = 0; index < RefList.Count; index++)
        {
            var nextIndex = index + 1;
            if (nextIndex == RefList.Count) nextIndex = 0;

            var dict = RefList[index];
            var nextDict = RefList[nextIndex];

            foreach (var kvp in dict)
            {
                var joint = kvp.Key;
                var data = kvp.Value;
                var nextData = nextDict[joint];

                data.velocity = (nextData.position - data.position) / deltaTime;
                data.angularVelocity = GetAngularVelocity(data.rotation, nextData.rotation);
            }
        }

        _animatorRef.speed = 1f;

        int size = 5;
        int gap = 3;
        for (var i = 0; i < size; ++i)
        for (var j = 0; j < size; ++j)
            Instantiate(mannequinAgentPrefab, new Vector3(
                gap / 2 * (1 - size + 2 * i),
                0,
                gap * (-1 - j)
            ), Quaternion.identity, transform);
    }

    public int GetNextFrame(int frame)
    {
        var nextFrame = frame + 1;
        if (nextFrame >= RefList.Count)
            nextFrame = 0;
        return nextFrame;
    }

    public int GetFrame(float time)
    {
        while (time > animationLength)
            time -= animationLength;
        return (int) (time / deltaTime);
    }

    public TransformData GetTransformData(float time, RagdollJoint joint)
    {
        return GetTransformData(GetFrame(time), joint);
    }

    public TransformData GetTransformData(int frame, RagdollJoint joint)
    {
        return RefList[frame][joint];
    }

    internal static Vector3 GetAngularVelocity(Quaternion from, Quaternion to)
    {
        var q = to * Quaternion.Inverse(from);

        if (Mathf.Abs(q.w) > 1023.5f / 1024.0f)
            return new Vector3(0, 0, 0);

        float gain;
        if (q.w < 0.0f)
        {
            var angle = Mathf.Acos(-q.w);
            gain = -2.0f * angle / (Mathf.Sin(angle) * deltaTime);
        }
        else
        {
            var angle = Mathf.Acos(q.w);
            gain = 2.0f * angle / (Mathf.Sin(angle) * deltaTime);
        }

        return new Vector3(q.x * gain, q.y * gain, q.z * gain);
    }
}