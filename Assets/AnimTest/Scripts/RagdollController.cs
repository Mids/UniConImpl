using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;

// #define RECORD_ACTION

public class RagdollController : MonoBehaviour
{
    private List<RagdollJoint> _abs;
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
            _abs[i].ResetJoint(skeleton.joints[0].rotation, skeleton.joints[i]);
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
        }

        forcePenalty /= actionsArrayLength;
        actionsArray.CopyTo(_lastActionsArray, 0);

        for (var i = 0; i < actionsArrayLength; i++)
            actionsArray[i] = actionsArray[i] * actionsArray[i] * actionsArray[i] * actionsArray[i] * actionsArray[i];

        // 3 * 10 + 1 * 4 = 34
        var actionIndex = 0;
        foreach (var ab in _abs)
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

    private void Awake()
    {
        _abs = GetComponentsInChildren<RagdollJoint>().ToList();
        _size = _abs.Count;
        SetParents();
#if UNITY_EDITOR && RECORD_ACTION
        _sw = new StreamWriter("actionOutput.txt");
#endif // UNITY_EDITOR && RECORD_ACTION
    }

    private void SetParents()
    {
        _abs[1].SetRootAndParent(_abs[0], _abs[0]);
        _abs[2].SetRootAndParent(_abs[0], _abs[1]);
        _abs[3].SetRootAndParent(_abs[0], _abs[2]);
        _abs[4].SetRootAndParent(_abs[0], _abs[0]);
        _abs[5].SetRootAndParent(_abs[0], _abs[4]);
        _abs[6].SetRootAndParent(_abs[0], _abs[5]);
        _abs[7].SetRootAndParent(_abs[0], _abs[0]);
        _abs[8].SetRootAndParent(_abs[0], _abs[7]);
        _abs[9].SetRootAndParent(_abs[0], _abs[8]);
        _abs[10].SetRootAndParent(_abs[0], _abs[9]);
        _abs[11].SetRootAndParent(_abs[0], _abs[10]);
        _abs[12].SetRootAndParent(_abs[0], _abs[8]);
        _abs[13].SetRootAndParent(_abs[0], _abs[8]);
        _abs[14].SetRootAndParent(_abs[0], _abs[13]);
        _abs[15].SetRootAndParent(_abs[0], _abs[14]);
    }

#if UNITY_EDITOR && RECORD_ACTION
    private void OnDestroy()
    {
        _sw.Close();
    }
#endif // UNITY_EDITOR && RECORD_ACTION
}