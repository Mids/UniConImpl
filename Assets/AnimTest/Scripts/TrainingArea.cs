using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;

public class TransformData
{
    public Vector3 position;
    public Quaternion rotation;
    public Vector3 localPosition;
    public Quaternion localRotation;
    public Vector3 velocity;
    public Vector3 angularVelocity;
    public Vector3 centerOfMass;
    public Vector3 comVelocity;

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

    [HideInInspector]
    public Animator _animatorRef;

    private Rigidbody[] _rigidbody;
    public float animationLength;

    public AnimationClip animationClip;

    private static readonly int Reset = Animator.StringToHash("Reset");

    public List<Dictionary<RagdollJoint, TransformData>> RefList = new List<Dictionary<RagdollJoint, TransformData>>();

    private float passedTime = 0f;
    public const float deltaTime = 0.017f;


    // Start is called before the first frame update
    private IEnumerator Start()
    {
        _animatorRef = mannequinRef.GetComponentInChildren<Animator>();
        _animatorRef.speed = 0f;
        _rigidbody = mannequinRef.GetComponentsInChildren<Rigidbody>();
        animationLength = animationClip.length;

        var refTransforms = _animatorRef.GetComponentsInChildren<Transform>();


        while (passedTime < animationLength)
        {
            _animatorRef.Play(animationClip.name, 0, passedTime / animationLength);

            yield return new WaitForEndOfFrame();

            var newDict = new Dictionary<RagdollJoint, TransformData>();

            foreach (var kvp in RagdollData.RagdollJointNames)
            {
                var t = refTransforms.FirstOrDefault(p => p.name.Contains(kvp.Value));
                if (t == default) continue;

                newDict[kvp.Key] = new TransformData(t);
            }

            newDict[RagdollJoint.Pelvis].centerOfMass = GetCenterOfMass(_rigidbody);

            RefList.Add(newDict);

            passedTime += deltaTime;
        }

        print($"Total frame: {RefList.Count}");

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

            dict[RagdollJoint.Pelvis].comVelocity =
                (nextDict[RagdollJoint.Pelvis].centerOfMass - dict[RagdollJoint.Pelvis].centerOfMass) / deltaTime;
        }

        _animatorRef.speed = 1f;
#if UNITY_EDITOR
        int size = 4;
#else
        int size = 5;
#endif
        int gap = 3;
        for (var i = 0; i < size; ++i)
        for (var j = 0; j < size; ++j)
            Instantiate(mannequinAgentPrefab, new Vector3(
                gap / 2 * (1 - size + 2 * i),
                0,
                gap * (-1 - j)
            ), Quaternion.identity, transform);
    }

    public Vector3 GetCenterOfMass(IEnumerable<Rigidbody> rigidbodies)
    {
        var mass = 0f;
        var centerOfMass = Vector3.zero;

        foreach (var rb in rigidbodies)
        {
            rb.ResetCenterOfMass();

            mass += rb.mass;
            centerOfMass += (rb.transform.position - mannequinRef.transform.position + rb.centerOfMass) * rb.mass;
        }

        centerOfMass /= mass;

        Assert.IsFalse(float.IsNaN(centerOfMass.sqrMagnitude), "float.IsNaN(centerOfMass.sqrMagnitude)");

        return centerOfMass;
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

        if (Mathf.Abs(q.w) > 0.999999f)
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