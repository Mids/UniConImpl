using System;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.Assertions;

[RequireComponent(typeof(BehaviorParameters))]
public class MannequinAgent : Agent
{
    private TrainingArea _trainingArea;

    private GameObject _mannequinRef;

    private RagdollController _ragdoll;

    private float _episodeTime = 0f;

    public override void Initialize()
    {
        base.Initialize();
        _trainingArea = GetComponentInParent<TrainingArea>();
        _mannequinRef = _trainingArea.mannequinRef;
        _ragdoll = GetComponent<RagdollController>();

        var transforms = _mannequinRef.GetComponentsInChildren<Transform>();
        var data = _ragdoll.RagdollDataDict.Values.ToArray();

        foreach (var t in transforms)
        {
            var datum = data.FirstOrDefault(p => t.name.Contains(p.Name));
            if (datum == default) continue;

            datum.RefTransform = t;
        }
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
         * MiddleSpine, // X, Y
         * Head, // X, Y
         * 2+1+2+1+2+1+2+1+2+2 = 16
         */
        var actionsArray = actions.ContinuousActions.Array;

        // print("actionReceived");

        Assert.IsTrue(actionsArray.Length == 16);

        for (var i = 0; i < actionsArray.Length; i++) actionsArray[i] *= 20;

        _ragdoll.AddTorque(RagdollJoint.LeftHips, actionsArray[0], actionsArray[1], 0f);
        _ragdoll.AddTorque(RagdollJoint.LeftKnee, actionsArray[2], 0f, 0f);
        _ragdoll.AddTorque(RagdollJoint.RightHips, actionsArray[3], actionsArray[4], 0f);
        _ragdoll.AddTorque(RagdollJoint.RightKnee, actionsArray[5], 0f, 0f);
        _ragdoll.AddTorque(RagdollJoint.LeftArm, actionsArray[6], actionsArray[7], 0f);
        _ragdoll.AddTorque(RagdollJoint.LeftElbow, actionsArray[8], 0f, 0f);
        _ragdoll.AddTorque(RagdollJoint.RightArm, actionsArray[9], actionsArray[10], 0f);
        _ragdoll.AddTorque(RagdollJoint.RightElbow, actionsArray[11], 0f, 0f);
        _ragdoll.AddTorque(RagdollJoint.MiddleSpine, actionsArray[12], actionsArray[13], 0f);
        _ragdoll.AddTorque(RagdollJoint.Head, actionsArray[14], actionsArray[15], 0f);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
    }

    public override void OnEpisodeBegin()
    {
        _episodeTime = 0f;
        _trainingArea.ResetArea();
        foreach (var ragdollData in _ragdoll.RagdollDataDict.Values)
        {
            ragdollData.RigidbodyComp.velocity = Vector3.zero;
            ragdollData.RigidbodyComp.angularVelocity = Vector3.zero;
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 16 * 11 = 176
        var pelvis = _ragdoll.RagdollDataDict[RagdollJoint.Pelvis];

        var pelvisRefPos = pelvis.RefTransform.localPosition;
        var pelvisAgentPos = pelvis.AgentTransform.localPosition;
        var pelvisDiff = pelvisRefPos - pelvisAgentPos;
        sensor.AddObservation(pelvisDiff);
        sensor.AddObservation(pelvis.GetPosition());
        sensor.AddObservation(pelvis.GetRotation());
        sensor.AddObservation(pelvis.GetVelocity());
        sensor.AddObservation(pelvis.GetAngularVelocity());
        var mag = pelvisDiff.magnitude;
        mag *= mag;
        AddReward((1f - mag) * 0.1f);

        // print("pelvis reward: " + (0.1f - Mathf.Abs(pelvisDiff)));

        foreach (RagdollJoint joint in Enum.GetValues(typeof(RagdollJoint)))
        {
            if (joint == RagdollJoint.Pelvis)
                continue;

            var data = _ragdoll.RagdollDataDict[joint];

            var refPos = data.RefTransform.position - pelvis.RefTransform.position;
            var agentPos = data.AgentTransform.position - pelvis.AgentTransform.position;

            var jointDiff = refPos - agentPos;

            sensor.AddObservation(jointDiff);
            sensor.AddObservation(data.GetPosition());
            sensor.AddObservation(data.GetRotation());
            sensor.AddObservation(data.GetVelocity());
            sensor.AddObservation(data.GetAngularVelocity());

            AddReward((1f - jointDiff.magnitude) * 0.01f);

            // print(joint + " reward: " + (0.2f - jointDiff.magnitude));
        }
    }

    // Update is called once per frame
    private void Update()
    {
        // 3 + 3 * 10 = 31
        var pelvis = _ragdoll.RagdollDataDict[RagdollJoint.Pelvis];

        var pelvisRefPos = pelvis.RefTransform.localPosition;
        var pelvisAgentPos = pelvis.AgentTransform.localPosition;
        var pelvisDiff = pelvisRefPos - pelvisAgentPos;
        if (Mathf.Abs(pelvisDiff.y) > 0.5f)
        {
            // print("pelvis too far :" + pelvisDiff.magnitude);

            AddReward(-10f);
            EndEpisode();
        }

        // print("pelvis reward: " + (0.1f - Mathf.Abs(pelvisDiff)));

        foreach (RagdollJoint joint in Enum.GetValues(typeof(RagdollJoint)))
        {
            if (joint == RagdollJoint.Pelvis)
                continue;

            var data = _ragdoll.RagdollDataDict[joint];

            var refPos = data.RefTransform.position - pelvis.RefTransform.position;
            var agentPos = data.AgentTransform.position - pelvis.AgentTransform.position;

            var jointDiff = refPos - agentPos;

            if (joint == RagdollJoint.Head)
                if (Mathf.Abs(jointDiff.y) > 0.5f)
                {
                    // print("head too far :" + jointDiff.magnitude);
                    AddReward(-10f);
                    EndEpisode();
                }

            // print(joint + " reward: " + (0.2f - jointDiff.magnitude));
        }

        _episodeTime += Time.deltaTime;
        if (_episodeTime > _trainingArea.AnimationLength)
        {
            AddReward(10f);
            EndEpisode();
        }
    }
}