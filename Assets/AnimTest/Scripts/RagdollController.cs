#define RECORD_ACTION

using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;

public class RagdollController : MonoBehaviour
{
    public float allStiffness;
    public float allDamping;

    private List<RagdollJoint> _joints;
    private int _size = 0;
    private float[] _lastActionsArray;
    private float[] _lastActionsDiffArray;

#if UNITY_EDITOR && RECORD_ACTION
    private StreamWriter _sw;
#endif // UNITY_EDITOR && RECORD_ACTION

    public void ResetRagdoll(SkeletonData skeleton)
    {
        for (var i = 0; i < _lastActionsArray?.Length; i++) _lastActionsArray[i] = 0f;
        for (var i = 0; i < _lastActionsDiffArray?.Length; i++) _lastActionsDiffArray[i] = 0f;

        if (skeleton.joints.Length != _size)
            Assert.AreEqual(skeleton.joints.Length, _size);

        for (var i = 0; i < _size; ++i)
        {
            _joints[i].stiffness = allStiffness;
            _joints[i].damping = allDamping;
            _joints[i].SetTargetJoint(skeleton.joints[i]);
            _joints[i].ResetJoint(skeleton.joints[0].rotation);
        }
    }

    public void SetPDTarget(SkeletonData skeleton)
    {
        for (var i = 0; i < _joints.Count; i++)
        {
            _joints[i].SetTargetJoint(skeleton.joints[i]);
            _joints[i].SetPDTarget();
        }
    }

    public float OnActionReceived(float[] actionsArray)
    {
        var forcePenalty = 0f;
        var actionsArrayLength = actionsArray.Length;
        _lastActionsArray ??= new float[actionsArrayLength];
        _lastActionsDiffArray ??= new float[actionsArrayLength];


#if UNITY_EDITOR && RECORD_ACTION
        var actionString = actionsArray.Aggregate("", (current, action) => current + (action + "\t"));
        _sw.WriteLine(actionString);
#endif // UNITY_EDITOR && RECORD_ACTION

        for (var i = 0; i < actionsArrayLength; i++)
        {
            var diff = actionsArray[i] - _lastActionsArray[i];
            if (_lastActionsDiffArray[i] * diff < 0)
                forcePenalty += diff * diff;
            _lastActionsDiffArray[i] = diff;
            _lastActionsArray[i] = actionsArray[i];
        }

        forcePenalty /= actionsArrayLength;

        // 3 * 10 + 1 * 4 = 34
        var actionIndex = 0;
        foreach (var ab in _joints)
        {
            switch (ab.dofCount)
            {
                case 1:
                    ab.AddRelativeTorque(actionsArray[actionIndex]);
                    break;
                case 3:
                    ab.AddRelativeTorque(actionsArray[actionIndex],
                        actionsArray[actionIndex + 1],
                        actionsArray[actionIndex + 2]);
                    break;
            }

            actionIndex += ab.dofCount;
        }

        return forcePenalty;
    }

    public void FreezeAll(bool b)
    {
        _joints[0].Freeze(b);
    }

    private void Awake()
    {
        _joints = GetComponentsInChildren<RagdollJoint>().ToList();
        _size = _joints.Count;
        SetParents();
#if UNITY_EDITOR && RECORD_ACTION
        _sw = new StreamWriter("actionOutput.txt");
#endif // UNITY_EDITOR && RECORD_ACTION
    }

    private void SetParents()
    {
        _joints[1].SetRootAndParent(_joints[0], _joints[0]);
        _joints[2].SetRootAndParent(_joints[0], _joints[1]);
        _joints[3].SetRootAndParent(_joints[0], _joints[2]);
        _joints[4].SetRootAndParent(_joints[0], _joints[0]);
        _joints[5].SetRootAndParent(_joints[0], _joints[4]);
        _joints[6].SetRootAndParent(_joints[0], _joints[5]);
        _joints[7].SetRootAndParent(_joints[0], _joints[0]);
        _joints[8].SetRootAndParent(_joints[0], _joints[7]);
        _joints[9].SetRootAndParent(_joints[0], _joints[8]);
        _joints[10].SetRootAndParent(_joints[0], _joints[9]);
        _joints[11].SetRootAndParent(_joints[0], _joints[10]);
        _joints[12].SetRootAndParent(_joints[0], _joints[8]);
        _joints[13].SetRootAndParent(_joints[0], _joints[8]);
        _joints[14].SetRootAndParent(_joints[0], _joints[13]);
        _joints[15].SetRootAndParent(_joints[0], _joints[14]);
    }

#if UNITY_EDITOR && RECORD_ACTION
    private void OnDestroy()
    {
        _sw.Close();
    }
#endif // UNITY_EDITOR && RECORD_ACTION
}