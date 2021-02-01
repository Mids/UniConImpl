using System;
using System.Collections;
using System.Collections.Generic;
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

    private Queue<Dictionary<RagdollJoint, Vector3>> _refPositions = new Queue<Dictionary<RagdollJoint, Vector3>>();

    private Queue<Dictionary<RagdollJoint, Quaternion>> _refRotations =
        new Queue<Dictionary<RagdollJoint, Quaternion>>();

    public int targetStatesLength = 3;


    public float rewardRootPos = 0.2f;
    public float rewardRootRot = 0.2f;
    public float rewardJointPos = 0.1f;
    public float rewardJointRot = 0.4f;
    public float rewardJointAngVel = 0.1f;

    private Vector3 _prevCenterOfMass;

    public override void Initialize()
    {
        base.Initialize();
        _trainingArea = GetComponentInParent<TrainingArea>();
        _ragdoll = GetComponent<RagdollController>();
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
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

        // print("actionReceived");

        Assert.IsTrue(actionsArray.Length == 12);

        for (var i = 0; i < actionsArray.Length; i++) actionsArray[i] *= 10;
        
        // print($"LeftKnee: {actionsArray[2]}");

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
        CalculateReward();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        for (var index = 0; index < actionsOut.ContinuousActions.Array.Length; index++)
        {
            actionsOut.ContinuousActions.Array[index] = Random.Range(-0.1f, 0.1f);
        }
    }

    public override void OnEpisodeBegin()
    {
        _episodeTime = 0f;
        // _initTime = _trainingArea._animatorRef.GetCurrentAnimatorStateInfo(0).normalizedTime % 1
        //             * _trainingArea.animationLength;
        _initTime = 0;
        _isStarted = false;
        _earlyTerminationStack = 0;
        _currentFrame = -1;

        foreach (var kvp in _ragdoll.RagdollDataDict)
        {
            var joint = kvp.Key;
            var ragdollData = kvp.Value;
            var transformData = _trainingArea.GetTransformData(_initTime, joint);

            ragdollData.AgentTransform.localPosition = transformData.localPosition;
            ragdollData.AgentTransform.localRotation = transformData.localRotation;
            ragdollData.RigidbodyComp.velocity = Vector3.zero;
            ragdollData.RigidbodyComp.angularVelocity = Vector3.zero;
        }

        _prevCenterOfMass = _trainingArea.GetTransformData(_initTime, RagdollJoint.Pelvis).centerOfMass;

        StartCoroutine(WaitForStart());
    }

    private IEnumerator WaitForStart()
    {
        yield return new WaitForFixedUpdate();
        _isStarted = true;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 193 + 600 = 793
        var animationTime = _initTime + _episodeTime;
        var currentFrame = _trainingArea.GetFrame(animationTime);
        var joints = Enum.GetValues(typeof(RagdollJoint));

        // Current State (15 * 13 - 2) = 193
        var root = _ragdoll.RagdollDataDict[RagdollJoint.Pelvis];
        sensor.AddObservation(root.GetLocalPosition().y);
        sensor.AddObservation(root.GetLocalRotation());
        sensor.AddObservation(root.GetVelocity());
        sensor.AddObservation(root.GetAngularVelocity());

        foreach (RagdollJoint joint in joints)
        {
            if (joint == RagdollJoint.Pelvis) continue;

            var data = _ragdoll.RagdollDataDict[joint];

            sensor.AddObservation(data.GetPosition() - root.GetPosition());
            sensor.AddObservation(data.GetRotation() * Quaternion.Inverse(root.GetRotation()));
            sensor.AddObservation(data.GetVelocity());
            sensor.AddObservation(data.GetAngularVelocity());
        }

        // Target State (15*13 - 2 + 3 + 4) * 3 = 600
        var stateFrame = currentFrame;
        for (int i = 0; i < targetStatesLength; ++i)
        {
            stateFrame = _trainingArea.GetNextFrame(stateFrame);

            var pelvis = _trainingArea.GetTransformData(stateFrame, RagdollJoint.Pelvis);

            // Root Trajectory 
            sensor.AddObservation(pelvis.position - root.GetPosition());
            sensor.AddObservation(pelvis.rotation * Quaternion.Inverse(root.GetRotation()));

            sensor.AddObservation(pelvis.localPosition.y);
            sensor.AddObservation(pelvis.localRotation);
            sensor.AddObservation(pelvis.velocity);
            sensor.AddObservation(pelvis.angularVelocity);

            foreach (RagdollJoint joint in joints)
            {
                if (joint == RagdollJoint.Pelvis) continue;

                var data = _trainingArea.GetTransformData(stateFrame, joint);

                sensor.AddObservation(data.position - pelvis.position);
                sensor.AddObservation(data.rotation * Quaternion.Inverse(pelvis.rotation));
                sensor.AddObservation(data.velocity);
                sensor.AddObservation(data.angularVelocity);
            }
        }
    }

    public void CalculateReward()
    {
        if (!_isStarted)
        {
            return;
        }
        
        var animationTime = _initTime + _episodeTime;
        var curFrame = _trainingArea.GetFrame(animationTime);
        if (_currentFrame == curFrame) return;
        
        _currentFrame = curFrame;

        _isTerminated = false;
        
        SetReward(0);

        // 193 + 600 = 793
        var joints = Enum.GetValues(typeof(RagdollJoint));

        // TODO: Recalculate reward function
        var root = _ragdoll.RagdollDataDict[RagdollJoint.Pelvis];
        var targetRoot = _trainingArea.GetTransformData(animationTime, RagdollJoint.Pelvis);

        var posReward = (root.GetLocalPosition() - targetRoot.localPosition).sqrMagnitude;

        var rotReward = Quaternion.Angle(root.GetLocalRotation(), targetRoot.localRotation) / 180 * Mathf.PI; // degrees
        rotReward *= rotReward;

        // Debug.DrawLine(transform.position, transform.position + _ragdoll.GetCenterOfMass(), Color.cyan, 0.1f);
        // Debug.DrawLine(transform.position, transform.position + targetRoot.centerOfMass, Color.blue, 0.1f);
        var comVelocity = (_ragdoll.GetCenterOfMass() - _prevCenterOfMass) / Time.fixedDeltaTime;
        var comReward = (comVelocity - targetRoot.comVelocity).sqrMagnitude;
        _prevCenterOfMass = _ragdoll.GetCenterOfMass();
        if (float.IsNaN(comReward))
        {
            print($"MannequinAgent's comReward: NaN");
            return;
        }

        // print($"Root posReward: {posReward}, rotReward: {rotReward}");

        foreach (RagdollJoint joint in joints)
        {
            if (joint != RagdollJoint.Head
                && joint != RagdollJoint.LeftHand
                && joint != RagdollJoint.RightHand
                && joint != RagdollJoint.LeftFoot
                && joint != RagdollJoint.RightFoot)
                continue;

            var data = _ragdoll.RagdollDataDict[joint];
            var targetData = _trainingArea.GetTransformData(animationTime, joint);

            var posDiff = data.GetPosition() - root.GetPosition() - (targetData.position - targetRoot.position);
            var jointPosReward = posDiff.sqrMagnitude / 5;
            posReward += jointPosReward;


            var rotDiff = Quaternion.Angle(data.GetRotation(), root.GetRotation()) -
                          Quaternion.Angle(targetData.rotation, targetRoot.rotation);
            var jointRotReward = Mathf.Abs(rotDiff) / 180 * Mathf.PI;
            jointRotReward *= jointRotReward / 5;
            rotReward += jointRotReward;

            
            // var jointAngVelReward = (data.GetAngularVelocity() - targetData.angularVelocity).magnitude / 5000;
            // if (jointAngVelReward > 0.01f) _isTerminated = true;
            // AddReward(0.01f - jointAngVelReward);
            
            // print($"{joint} posReward: {jointPosReward}, rotReward: {jointRotReward}");
        }
        
        posReward = Mathf.Exp(-2 * posReward);
        if (posReward < 0.3) _isTerminated = true;

        rotReward = Mathf.Exp(-2 * rotReward);
        if (rotReward < 0.3) _isTerminated = true;

        comReward = Mathf.Exp(-20 * comReward);
        // if (comReward < 0.1) _isTerminated = true;
        AddReward((posReward + rotReward + comReward)/3 - 0.2f);
        
        // print($"Total reward: {posReward} + {rotReward} + {comReward} = {posReward + rotReward + comReward}");

        if (_episodeTime > _trainingArea.animationLength)
        {
            _earlyTerminationStack = EarlyTerminationMax;
            _isTerminated = true;
        }
        
        if (_isTerminated)
        {
            EarlyTerminate();
        }
    }

    private void EarlyTerminate()
    {
        if (++_earlyTerminationStack < EarlyTerminationMax) return;

        // var timeRatio = _episodeTime / _trainingArea.animationLength;
        // timeRatio *= timeRatio;
        //
        // AddReward(-5f + 10 * timeRatio);
        EndEpisode();
    }
    
    private void Update()
    {
        cumulativeRewardText.text = GetCumulativeReward().ToString("0.00");
    }
}