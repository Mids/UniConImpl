using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using DataProcessor;
using TMPro;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.Assertions;
using Random = UnityEngine.Random;

[RequireComponent(typeof(BehaviorParameters))]
public class MannequinAgent : Agent
{
    public TextMeshPro cumulativeRewardText;

    private TrainingArea _trainingArea;

    private RagdollController _ragdoll;

    private float _episodeTime = 0f;
    private float _initTime = 0f;
    private bool _isStarted = false;
    private bool _isTerminated = false;
    private int _earlyTerminationStack = 0;
    private const int EarlyTerminationMax = 1;
    private int _currentFrame = -1;
    private float _lastReward = 0;
    private bool _isRewarded = false;

    public int targetStatesLength = 3;
    private static readonly int[] FrameOffset = {1, 4, 16};


    public float rewardRootPos = 0.2f;
    public float rewardRootRot = 0.2f;
    public float rewardJointPos = 0.1f;
    public float rewardJointRot = 0.4f;
    public float rewardJointAngVel = 0.1f;

    private Vector3 _prevCenterOfMass;

    private MotionData _currentMotion;
    public GameObject AgentMesh;
    public AnimationClip animClip;
    public float fps = 30f;

    [Header("Reference")]
    public Animator RefAnimator;

    public AnimatorOverrideController tOverrideController;
    public Dictionary<string, Transform> RefTransformDict = new Dictionary<string, Transform>();


    // private StreamWriter sw;

    public override void Initialize()
    {
        base.Initialize();
        _trainingArea = GetComponentInParent<TrainingArea>();
        _ragdoll = GetComponent<RagdollController>();
        // sw = new StreamWriter("reward.txt");

        var refTransforms = RefAnimator.GetComponentsInChildren<Transform>();
        foreach (var refTransform in refTransforms) RefTransformDict[refTransform.name] = refTransform;
    }

    public float power;

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (!_isStarted) return;
        /***
         * LeftHips, // X, Y
         * LeftKnee, // X
         * RightHips, // X, Y
         * RightKnee, // X
         * LeftArm, // X, Y
         * LeftElbow, // X
         * RightArm, // X, Y
         * RightElbow, // X
         * 2+1+2+1+2+1+2+1 = 12
         */
        var actionsArray = actions.ContinuousActions.Array;

        Assert.IsTrue(actionsArray.Length == 12);

        var forcePenalty = 0f;
        for (var i = 0; i < actionsArray.Length; i++)
        {
            forcePenalty += actionsArray[i] * actionsArray[i];
            actionsArray[i] *= 10;
        }

        // if (power != 0)
        // {
        //     actionsArray[2] = power;
        //     actionsArray[5] = power;
        // }
        // print($"right knee : {actionsArray[5]}");

        forcePenalty /= actionsArray.Length;

        _ragdoll.AddTorque(RagdollJoint.LeftHips, actionsArray[0], actionsArray[1], 0f);
        _ragdoll.AddTorque(RagdollJoint.LeftKnee, actionsArray[2], 0f, 0f);
        _ragdoll.AddTorque(RagdollJoint.RightHips, actionsArray[3], actionsArray[4], 0f);
        _ragdoll.AddTorque(RagdollJoint.RightKnee, actionsArray[5], 0f, 0f);
        _ragdoll.AddTorque(RagdollJoint.LeftArm, actionsArray[6], actionsArray[7], 0f);
        _ragdoll.AddTorque(RagdollJoint.LeftElbow, actionsArray[8], 0f, 0f);
        _ragdoll.AddTorque(RagdollJoint.RightArm, actionsArray[9], actionsArray[10], 0f);
        _ragdoll.AddTorque(RagdollJoint.RightElbow, actionsArray[11], 0f, 0f);
        // _ragdoll.AddTorque(RagdollJoint.MiddleSpine, actionsArray[12], actionsArray[13], 0f);
        // _ragdoll.AddTorque(RagdollJoint.Head, actionsArray[14], actionsArray[15], 0f);

        _episodeTime += Time.fixedDeltaTime;

        SetReward(0);
        AddTargetStateReward();
        AddReward((1f - forcePenalty) / 1000);
#if UNITY_EDITOR
        ShowReward();
#endif
    }

    private void OnDestroy()
    {
        // sw.Close();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        for (var index = 0; index < actionsOut.ContinuousActions.Array.Length; index++)
            actionsOut.ContinuousActions.Array[index] = Random.Range(-0.1f, 0.1f);
    }

    public override void OnEpisodeBegin()
    {
        _episodeTime = 0f;
        _initTime = 0;
        _isStarted = false;
        _earlyTerminationStack = 0;
        _currentFrame = -1;
#if UNITY_EDITOR
        _lastReward = 0;
#endif

        // sw.WriteLine("\n");

        var motionPair = _trainingArea.GetRandomMotion();
        var motionKey = motionPair.Key;

        animClip = _trainingArea.GetAnimationClip(motionKey);
        _currentMotion = motionPair.Value;

        // RSI
        _initTime = Random.Range(0f, animClip.length / 2);

        ResetRefPose(animClip, _initTime);
        CopyRefPose();

        foreach (var ragdollData in _ragdoll.RagdollDataDict.Select(kvp => kvp.Value))
        {
            ragdollData.RigidbodyComp.position = ragdollData.GetPosition();
            ragdollData.RigidbodyComp.rotation = ragdollData.GetRotation();
            ragdollData.RigidbodyComp.velocity = Vector3.zero;
            ragdollData.RigidbodyComp.angularVelocity = Vector3.zero;
        }

        StartCoroutine(WaitForStart());
    }

    private void ResetRefPose(AnimationClip clip, float time)
    {
        var t = RefAnimator.transform;

        t.localPosition = Vector3.zero;
        t.localRotation = Quaternion.identity;

        if (tOverrideController == default)
        {
            tOverrideController = new AnimatorOverrideController(RefAnimator.runtimeAnimatorController);
            RefAnimator.runtimeAnimatorController = tOverrideController;
        }

        tOverrideController["Handshake_001"] = clip;
        RefAnimator.Play("Handshake_001", 0, time / clip.length);
        RefAnimator.speed = 1;
    }

    private void CopyRefPose()
    {
        var transforms = AgentMesh.GetComponentsInChildren<Transform>();
        foreach (var t in transforms)
        {
            if (!RefTransformDict.ContainsKey(t.name)) continue;
            var refT = RefTransformDict[t.name];
            t.localPosition = refT.localPosition;
            t.localRotation = refT.localRotation;
        }
    }

    private IEnumerator WaitForStart()
    {
        yield return new WaitForEndOfFrame();
        _isStarted = true;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (!_isStarted) return;
        // 195 + 1 + 195 = 391
        var animationTime = _initTime + _episodeTime;
        var currentFrame = (int) (animationTime * 30f);
        var joints = Enum.GetValues(typeof(RagdollJoint));

        // Current State (15 * 13) = 195
        var root = _ragdoll.RagdollDataDict[RagdollJoint.Pelvis];
        var rootInv = Quaternion.Inverse(root.GetRotation());

        sensor.AddObservation(root.GetLocalPosition());
        sensor.AddObservation(root.GetLocalRotation());
        sensor.AddObservation(root.GetVelocity());
        sensor.AddObservation(root.GetAngularVelocity());

        foreach (RagdollJoint joint in joints)
        {
            if (joint == RagdollJoint.Pelvis) continue;

            var data = _ragdoll.RagdollDataDict[joint];
            sensor.AddObservation(rootInv * (data.GetPosition() - root.GetPosition()));
            sensor.AddObservation(rootInv * data.GetRotation());
            sensor.AddObservation(data.GetVelocity() - root.GetVelocity());
            sensor.AddObservation(data.GetAngularVelocity() - root.GetAngularVelocity());
        }

        // Time to next frame (1)
        sensor.AddObservation(currentFrame + 1 - animationTime * 30f);
        
        // Target State (5*13) * 3 = 195
        for (var i = 0; i < targetStatesLength; ++i)
        {
            var targetPose = _currentMotion.GetPose(currentFrame + FrameOffset[i]);

            foreach (var bone in SkeletonData.allBones)
            {
                sensor.AddObservation(targetPose[bone].Position);
                sensor.AddObservation(targetPose[bone].Rotation);
                sensor.AddObservation(targetPose[bone].Velocity);
                sensor.AddObservation(targetPose[bone].AngularVelocity);
            }
        }
    }


    public void AddTargetStateReward()
    {
        if (!_isStarted) return;

        var animationTime = _initTime + _episodeTime;
        var curFrame = (int) (animationTime * 30f);
        var targetPose = _currentMotion.GetPose(curFrame);
        if (_currentFrame == curFrame) return;

        _currentFrame = curFrame;

        _isTerminated = false;

        // 193 + 600 = 793
        var joints = Enum.GetValues(typeof(RagdollJoint));

        // TODO: Recalculate reward function
        var root = _ragdoll.RagdollDataDict[RagdollJoint.Pelvis];
        var targetRoot = targetPose.Root;
        var rootInv = Quaternion.Inverse(root.GetLocalRotation());

        var posReward = (root.GetLocalPosition() - targetRoot.Position).sqrMagnitude;

        var rotReward = Quaternion.Angle(root.GetLocalRotation(), targetRoot.Rotation) / 180 * Mathf.PI; // degrees
        rotReward *= rotReward;

        var velReward = (root.GetVelocity() - targetRoot.Velocity).sqrMagnitude;

        var avReward = (root.GetAngularVelocity() - targetRoot.AngularVelocity).sqrMagnitude;

        // print($"Root posReward: {posReward}, rotReward: {rotReward}");

        var totalJointPosReward = 0f;
        var totalJointRotReward = 0f;
        var totalJointVelReward = 0f;
        var totalJointAvReward = 0f;
        foreach (RagdollJoint joint in joints)
        {
            HumanBodyBones bone;
            switch (joint)
            {
                case RagdollJoint.LeftFoot:
                    bone = HumanBodyBones.LeftFoot;
                    break;
                case RagdollJoint.RightFoot:
                    bone = HumanBodyBones.RightFoot;
                    break;
                // case RagdollJoint.LeftHand:
                //     bone = HumanBodyBones.LeftHand;
                //     break;
                // case RagdollJoint.RightHand:
                //     bone = HumanBodyBones.RightHand;
                //     break;
                default:
                    continue;
            }

            var data = _ragdoll.RagdollDataDict[joint];
            var targetData = targetPose[bone];

            var posDiff = rootInv * (data.GetPosition() - root.GetPosition()) - targetData.Position;
            var jointPosReward = posDiff.sqrMagnitude;
            totalJointPosReward += jointPosReward / 8;


            var rotDiff = Quaternion.Angle(rootInv * data.GetRotation(), targetData.Rotation);
            var jointRotReward = Mathf.Abs(rotDiff) / 180 * Mathf.PI;
            jointRotReward *= jointRotReward;
            totalJointRotReward += jointRotReward / 2;

            var velDiff = rootInv * (data.GetVelocity() - root.GetVelocity()) - targetData.Velocity;
            var jointVelReward = velDiff.sqrMagnitude;
            totalJointVelReward += jointVelReward / 8;

            var avDiff = data.GetAngularVelocity() - root.GetAngularVelocity() - targetData.AngularVelocity;
            var jointAvReward = avDiff.sqrMagnitude;
            totalJointAvReward += jointAvReward / 2;
        }

#if UNITY_EDITOR
        // print($"Total reward: {posReward} + {rotReward} + {velReward} + {avReward}\n" +
        //       $"Joint reward : {totalJointPosReward} + {totalJointRotReward} + {totalJointVelReward}");
#endif

        // sw.WriteLine($"{posReward}\t{rotReward}\t{velReward}\t{avReward}");

        posReward += totalJointPosReward / 10f;
        rotReward += totalJointRotReward / 10f;
        velReward += totalJointVelReward / 10f;
        avReward += totalJointAvReward / 10f;
        
        

        posReward = Mathf.Exp(-2 * posReward);
        rotReward = Mathf.Exp(rotReward / -2);
        velReward = Mathf.Exp(velReward / -2);
        avReward = Mathf.Exp(avReward / -100);

        // var totalReward = (posReward + rotReward + velReward / 2 + avReward / 2) / 1.5f - 1f;
        var totalReward = (posReward + rotReward) / 1f - 1f;
// #if !UNITY_EDITOR
        if (posReward < 0.2)
        {
            _isTerminated = true;
            totalReward = -1;
        }

// #endif
        _isRewarded = true;
        AddReward(totalReward);

        if (animationTime > animClip.length)
        {
            _earlyTerminationStack = EarlyTerminationMax;
            _isTerminated = true;
        }

        if (_isTerminated) EarlyTerminate();
    }

    private void EarlyTerminate()
    {
        if (++_earlyTerminationStack < EarlyTerminationMax) return;

        _isStarted = false;
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
}