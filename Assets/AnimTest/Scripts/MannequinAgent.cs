using System.Collections;
using System.Collections.Generic;
using System.Linq;
using TMPro;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;

[RequireComponent(typeof(BehaviorParameters))]
public class MannequinAgent : Agent
{
    public TextMeshPro cumulativeRewardText;

    private TrainingArea _trainingArea;

#if UNITY_EDITOR
    private AnimationPlayer _RefAP;
#endif // UNITY_EDITOR

    private float _episodeTime = 0f;
    private float _initTime = 0f;
    private int _initFrame = 0;
    private bool _isTerminated = false;
    private int _earlyTerminationStack = 0;
    private const int EarlyTerminationMax = 1;
    private int _currentFrame = -1;
    private float _lastReward = 0;
    private bool _isRewarded = false;
    private bool _isInitialized = false;
    private Vector3 _lastCOM = Vector3.zero;

    private static readonly int[] FrameOffset = {1, 4, 16};


    // public float rewardRootPos = 0.2f;
    // public float rewardRootRot = 0.2f;
    // public float rewardJointPos = 0.1f;
    // public float rewardJointRot = 0.4f;
    // public float rewardJointAngVel = 0.1f;
    //
    // private Vector3 _prevCenterOfMass;

    public MotionData currentMotion;
    private SkeletonData currentPose => currentMotion.data[_currentFrame];
    private SkeletonData _initPose;
    public GameObject AgentMesh;
    public float fps = 30f;

    public List<Transform> AgentTransforms;
    public List<ArticulationBody> AgentABs;
    public List<FootContact> Feet;
    public RagdollController ragdollController;

    // public AnimationPlayer RefModel;
    // private Transform RefTransform;
    // private List<Transform> RefTransforms;

    public override void Initialize()
    {
        base.Initialize();
        _trainingArea = GetComponentInParent<TrainingArea>();
#if UNITY_EDITOR
        _RefAP = GetComponentInChildren<AnimationPlayer>();
#endif // UNITY_EDITOR

        ragdollController = GetComponent<RagdollController>();

        AgentABs = AgentMesh.GetComponentsInChildren<ArticulationBody>().ToList();
        AgentTransforms = AgentABs.Select(p => p.transform).ToList();
        Feet = AgentMesh.GetComponentsInChildren<FootContact>().ToList();
    }

    public override void OnEpisodeBegin()
    {
        _isInitialized = false;
        _episodeTime = 0f;
        _earlyTerminationStack = 0;

#if UNITY_EDITOR
        _lastReward = 0;
#endif // UNITY_EDITOR

        var motionIndex = _trainingArea.GetRandomMotionIndex();
        currentMotion = _trainingArea.GetMotion(motionIndex);
#if UNITY_EDITOR
        _RefAP.SetMotion(currentMotion);
#endif // UNITY_EDITOR

        // TODO: RSI
        _initFrame = Random.Range(0, _trainingArea.GetMotionTotalFrame(motionIndex) / 2);
        // _initFrame = 0;
        _initTime = _initFrame / fps;
        _currentFrame = _initFrame;

        _initPose = currentPose;
        Feet.ForEach(p => p.isContact = false);
        _lastCOM = _initPose.joints[0].position + _initPose.centerOfMass;

        ResetAgentPose();
        StartCoroutine(WaitForInit());
    }

    private IEnumerator WaitForInit()
    {
        yield return new WaitForEndOfFrame();
        _episodeTime = 0f;
        ragdollController.ResetRagdoll(_initPose);
        ragdollController.FreezeAll(false);
        RequestDecision();
        _isInitialized = true;
    }

    private void ResetAgentPose()
    {
        ragdollController.FreezeAll(true);
        ragdollController.ResetRagdoll(_initPose);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 191 + 573 = 764
        // Current State 11 + 180 = 191
        if (!ragdollController.OnCollectObservations(sensor))
            _isTerminated = true;

        // Target State (11 + 180) * 3 = 573
        foreach (var offset in FrameOffset)
        {
            var targetFrame = Mathf.Min(_currentFrame + offset, currentMotion.data.Count - 1);
            var targetPose = currentMotion.data[targetFrame];

            ragdollController.AddTargetObservation(sensor, targetPose);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var forcePenalty = ragdollController.OnActionReceived(actions.ContinuousActions.Array);

        SetReward(0);
        AddTargetStateReward();
        AddReward((0.5f - forcePenalty) / 10);
#if UNITY_EDITOR
        // print($"Force penalty : {(0.5f - forcePenalty) / 10}");
        ShowReward();
#endif
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        for (var index = 0; index < actionsOut.ContinuousActions.Array.Length; index++)
            actionsOut.ContinuousActions.Array[index] = Random.Range(-1f, 1f);
    }


    public void AddTargetStateReward()
    {
        var targetPose = currentPose;

        _isTerminated = false;

        var root = AgentTransforms[0];
        var rootAB = AgentABs[0];
        var rootParent = root.parent;
        var parentRot = rootParent.rotation;
        var rootPos = rootParent.localPosition + parentRot * root.localPosition;
        var rootRot = parentRot * root.localRotation;
        var rootInv = Quaternion.Inverse(rootRot);

        var targetRoot = targetPose.joints[0];

        var posReward = rootPos.y - targetRoot.position.y;
        posReward *= posReward;
        // var posReward = (rootPos - targetRoot.position).sqrMagnitude;

        var rotReward = Quaternion.Angle(rootRot, targetRoot.rotation) / 180 * Mathf.PI; // degrees
        rotReward *= rotReward;

        var velReward = (rootAB.velocity - targetRoot.velocity).sqrMagnitude;

        var avReward = (rootAB.angularVelocity - targetRoot.angularVelocity).sqrMagnitude;

        // print($"Root posReward: {posReward}, rotReward: {rotReward}");

        var totalJointPosReward = 0f;
        var totalJointRotReward = 0f;
        var totalJointVelReward = 0f;
        var totalJointAvReward = 0f;
        var jointNum = AgentABs.Count;
        for (var index = 1; index < jointNum; ++index)
        {
            // if (index != 3 &&
            //     index != 6 &&
            //     index != 11 &&
            //     index != 15) continue;
            var jointT = AgentTransforms[index];
            var jointAB = AgentABs[index];
            var targetData = targetPose.joints[index];

            var posDiff = rootInv * (jointT.position - root.position) - targetData.position;
            var jointPosReward = posDiff.sqrMagnitude;
            totalJointPosReward += jointPosReward;

            var rotDiff = Quaternion.Angle(rootInv * jointT.rotation, targetData.rotation);
            var jointRotReward = Mathf.Abs(rotDiff) / 180 * Mathf.PI;
            jointRotReward *= jointRotReward;
            totalJointRotReward += jointRotReward;

            var velDiff = rootInv * (jointAB.velocity - rootAB.velocity) - targetData.velocity;
            var jointVelReward = velDiff.sqrMagnitude;
            totalJointVelReward += jointVelReward;

            var avDiff = jointAB.angularVelocity - rootAB.angularVelocity - targetData.angularVelocity;
            var jointAvReward = avDiff.sqrMagnitude;
            totalJointAvReward += jointAvReward;
        }

        var comReward = GetComReward();

#if UNITY_EDITOR
        // print($"Total reward: {posReward} + {rotReward} + {velReward} + {avReward}\n" +
        //       $"Joint reward : {totalJointPosReward} + {totalJointRotReward} + {totalJointVelReward}");
#endif

        posReward += totalJointPosReward / 64f;
        // rotReward += totalJointRotReward / 16f;
        velReward += totalJointVelReward / 16f;
        avReward += totalJointAvReward / 16f;


        posReward = Mathf.Exp(posReward * -4f);
        rotReward = Mathf.Exp(rotReward / -4f);
        velReward = Mathf.Exp(velReward / -10f);
        avReward = Mathf.Exp(avReward / -20f);
        comReward = Mathf.Exp(comReward * -1f);

#if UNITY_EDITOR
        // print($"Total reward: {posReward} + {rotReward} + {velReward} + {avReward} + {comReward}\n");
#endif
        // var totalReward = (posReward + rotReward + velReward / 2 + avReward / 2) / 1.5f - 1f;

        var curHead = AgentTransforms[12].position - rootParent.parent.position;
        var targetHead = targetRoot.position + targetRoot.rotation * targetPose.joints[12].position;
        var distHead = (targetHead - curHead).magnitude;

        if (distHead > 1f)
            _isTerminated = true;

        var totalReward = comReward * (posReward + rotReward / 3 + velReward / 3 + (1f - distHead) / 3) - 1.05f;

        _isRewarded = true;

        AddReward(totalReward);

        if (_currentFrame == currentMotion.data.Count - 1)
        {
            _earlyTerminationStack = EarlyTerminationMax;
            _isTerminated = true;
        }

        if (_isTerminated) EarlyTerminate();
    }

    private float GetComReward()
    {
        var root = AgentTransforms[0];
        var rootParent = root.parent;
        var parentRot = rootParent.rotation;
        var rootPos = rootParent.localPosition + parentRot * root.localPosition;

        var com = Vector3.zero;
        var totalMass = 0f;
        foreach (var jointAB in AgentABs)
        {
            var mass = jointAB.mass;
            com += (jointAB.worldCenterOfMass - root.position) * mass;
            totalMass += mass;
        }

        com /= totalMass;
        com += rootPos;
        var comDir = com - _lastCOM;
        _lastCOM = com;
        var projectedCom = com;
        projectedCom.y = 0f;
        comDir.y = 0f;

        var lFootPos = Feet[0].transform.position;
        lFootPos.y = 0;
        var rFootPos = Feet[1].transform.position;
        rFootPos.y = 0;
        var lToePos = lFootPos + Feet[0].transform.rotation * new Vector3(0, 0.2f, 0);
        lToePos.y = 0;
        var rToePos = rFootPos + Feet[1].transform.rotation * new Vector3(0, 0.2f, 0);
        rToePos.y = 0;
        var comReward = 0f;

        if (Feet[0].isContact && Feet[1].isContact)
        {
            var isIn = PointInTriangle(projectedCom, lFootPos, lToePos, rFootPos)
                       || PointInTriangle(projectedCom, lFootPos, lToePos, rToePos)
                       || PointInTriangle(projectedCom, lFootPos, rFootPos, rToePos)
                       || PointInTriangle(projectedCom, lToePos, rFootPos, rToePos);
            var centerPos = lFootPos + rFootPos + lToePos + rToePos;
            centerPos /= 4f;
            var centerDir = centerPos - projectedCom;
            var angle = Vector3.Angle(comDir, centerDir);
            if (!isIn && angle > 90f)
                comReward = angle * Mathf.Deg2Rad;
        }
        else if (Feet[0].isContact)
        {
            var rFootVel = AgentABs[6].velocity;
            rFootVel.y = 0;
            var angle = Vector3.Angle(comDir, rFootVel);
            comReward = angle * Mathf.Deg2Rad;
        }
        else if (Feet[1].isContact)
        {
            var lFootVel = AgentABs[3].velocity;
            lFootVel.y = 0;
            var angle = Vector3.Angle(comDir, lFootVel);
            comReward = angle * Mathf.Deg2Rad;
        }


        return comReward;
    }

    private static float Sign(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        return (p1.x - p3.x) * (p2.z - p3.z) - (p2.x - p3.x) * (p1.z - p3.z);
    }

    private static bool PointInTriangle(Vector3 pt, Vector3 v1, Vector3 v2, Vector3 v3)
    {
        var d1 = Sign(pt, v1, v2);
        var d2 = Sign(pt, v2, v3);
        var d3 = Sign(pt, v3, v1);

        var hasNeg = d1 < 0 || d2 < 0 || d3 < 0;
        var hasPos = d1 > 0 || d2 > 0 || d3 > 0;

        return !(hasNeg && hasPos);
    }

    private void EarlyTerminate()
    {
        if (++_earlyTerminationStack < EarlyTerminationMax) return;

        EndEpisode();
    }

    private void ShowReward()
    {
        if (!_isRewarded) return;
        _isRewarded = false;
        var delta = GetCumulativeReward() - _lastReward;
        cumulativeRewardText.text = delta.ToString("0.00");
        _lastReward = GetCumulativeReward();
    }


    private void FixedUpdate()
    {
        if (!_isInitialized) return;

        _episodeTime += Time.fixedDeltaTime;
        var animationTime = _initTime + _episodeTime;
        var curFrame = (int) (animationTime * fps);

        if (curFrame != _currentFrame)
        {
            _currentFrame = curFrame;
            RequestDecision();

#if UNITY_EDITOR
            _RefAP.SetPose(currentPose);
#endif // UNITY_EDITOR
        }

        ragdollController.ApplyTorque();
    }
}