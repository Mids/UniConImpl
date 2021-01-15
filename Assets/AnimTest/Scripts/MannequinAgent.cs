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
        var data = RagdollData.RagdollDataDict.Values.ToArray();

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
        var actions = actionsOut.ContinuousActions.Array;
        for (var index = 0; index < actions.Length; index++)
        {
            actions[index] = 1f;
        }
    }

    public override void OnEpisodeBegin()
    {
        _episodeTime = 0f;
        _trainingArea.ResetArea();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1 + 3 * 10 = 31
        var pelvis = RagdollData.RagdollDataDict[RagdollJoint.Pelvis];

        var pelvisRefPos = pelvis.RefTransform.position;
        var pelvisAgentPos = pelvis.AgentTransform.position;
        var pelvisDiff = pelvisRefPos.y - pelvisAgentPos.y;
        sensor.AddObservation(pelvisDiff);
        AddReward(0.1f - Mathf.Abs(pelvisDiff));
        
        // print("pelvis reward: " + (0.1f - Mathf.Abs(pelvisDiff)));
        
        foreach (RagdollJoint joint in Enum.GetValues(typeof(RagdollJoint)))
        {
            if (joint == RagdollJoint.Pelvis)
                continue;

            var data = RagdollData.RagdollDataDict[joint];

            var refPos = data.RefTransform.position - pelvisRefPos;
            var agentPos = data.AgentTransform.position - pelvisAgentPos;

            var jointDiff = refPos - agentPos;
            
            sensor.AddObservation(jointDiff);
            AddReward(0.2f - jointDiff.magnitude);
            
            // print(joint + " reward: " + (0.2f - jointDiff.magnitude));
        }
    }

    // Update is called once per frame
    private void Update()
    {
        _episodeTime += Time.deltaTime;
        if (_episodeTime > _trainingArea.AnimationLength)
        {
            print("end episode");
            EndEpisode();
        }
    }
}