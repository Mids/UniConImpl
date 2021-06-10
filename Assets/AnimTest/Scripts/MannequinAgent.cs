using System.Collections.Generic;
using System.Linq;
using TMPro;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.Assertions;

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

    public int targetStatesLength = 3;
    private static readonly int[] FrameOffset = {1, 4, 16};


    public float rewardRootPos = 0.2f;
    public float rewardRootRot = 0.2f;
    public float rewardJointPos = 0.1f;
    public float rewardJointRot = 0.4f;
    public float rewardJointAngVel = 0.1f;

    private Vector3 _prevCenterOfMass;

    public MotionData currentMotion;
    private SkeletonData _initPose;
    private Vector3 _initRootPosition;
    private Quaternion _initRootRotation;
    private Quaternion _initRootRotationInv;
    public GameObject AgentMesh;
    public float fps = 30f;

    public List<Transform> AgentTransforms;
    public List<ArticulationBody> AgentABs;

    public float forceMultiplier;

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

        // RefTransform = _trainingArea.mannequinRef.transform;
        // var refABs = RefTransform.GetComponentsInChildren<ArticulationBody>().ToList();
        // RefTransforms = refABs.Select(p => p.transform).ToList();
        // RefModel = _trainingArea.mannequinRef.GetComponent<AnimationPlayer>();

        AgentABs = AgentMesh.GetComponentsInChildren<ArticulationBody>().ToList();
        AgentTransforms = AgentABs.Select(p => p.transform).ToList();
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
         *
         * 15*3=45a
         */
        var actionsArray = actions.ContinuousActions.Array;

        Assert.IsTrue(actionsArray.Length == AgentABs.Count * 3 - 3);

        var forcePenalty = 0f;
        var actionsArrayLength = actionsArray.Length;
        for (var i = 0; i < actionsArrayLength; i++)
        {
            forcePenalty += actionsArray[i] * actionsArray[i];
            actionsArray[i] *= forceMultiplier;
        }

        forcePenalty /= actionsArrayLength;

        for (var i = 1; i < AgentABs.Count; ++i)
            AgentABs[i].AddRelativeTorque(new Vector3(
                actionsArray[i * 3 - 3],
                actionsArray[i * 3 - 2],
                actionsArray[i * 3 - 1]));


        SetReward(0);
        AddTargetStateReward();
        AddReward((1f - forcePenalty) / 1000);
#if UNITY_EDITOR
        print(new Vector3(actionsArray[6], actionsArray[7], actionsArray[8]));
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
            actionsOut.ContinuousActions.Array[index] = Random.Range(-1f, 1f);
    }

    public override void OnEpisodeBegin()
    {
        _episodeTime = 0f;
        _earlyTerminationStack = 0;
        _currentFrame = -1;
#if UNITY_EDITOR
        _lastReward = 0;
#endif // UNITY_EDITOR

        var motionIndex = _trainingArea.GetRandomMotionIndex();
        currentMotion = _trainingArea.GetMotion(motionIndex);
#if UNITY_EDITOR
        _RefAP.motion = currentMotion;
        _RefAP.StopAllCoroutines();
        _RefAP.PlayCoroutine();
#endif // UNITY_EDITOR

        // TODO: RSI
        // _initFrame = Random.Range(0, _trainingArea.GetMotionTotalFrame(motionIndex));
        _initTime = 0f;
        _initFrame = 0;
        _currentFrame = _initFrame;

        _initPose = currentMotion.data[_currentFrame];
        _initRootPosition = transform.position + new Vector3(0, _initPose.joints[0].position.y, 0);
        _initRootRotation = _initPose.joints[0].rotation;
        _initRootRotationInv = Quaternion.Inverse(_initRootRotation);

        // ResetRefPose();
        ResetAgentPose();
    }

    // private void ResetRefPose()
    // {
    //     RefModel.motion = _currentMotion;
    //     RefModel.PlayCoroutine();
    // }

    private void ResetAgentPose()
    {
        // AgentABs.ForEach(p=>p.enabled = false);
        AgentABs[0].TeleportRoot(_initRootPosition, _initRootRotation);
        // AgentABs[0].ResetInertiaTensor();
        AgentABs[0].velocity = Vector3.zero;
        AgentABs[0].angularVelocity = Vector3.zero;

        for (int i = 1; i < AgentTransforms.Count; ++i)
        {
            // AgentABs[i].ResetInertiaTensor();
            // AgentABs[i].anchorRotation = Quaternion.identity;
            AgentABs[i].jointPosition = new ArticulationReducedSpace(0, 0, 0);
            AgentABs[i].velocity = Vector3.zero;
            AgentABs[i].angularVelocity = Vector3.zero;
        }

        // AgentABs.ForEach(p=>p.enabled =true);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // TODO:
        // 208+624 = 832
        var currentFrame = _currentFrame;
        var jointNum = AgentABs.Count;

        // Current State (16 * 13) = 208
        var rootAB = AgentABs[0];
        var root = AgentTransforms[0];
        var rootRot = root.localRotation;
        var rootInv = Quaternion.Inverse(rootRot);

        sensor.AddObservation(root.localPosition);
        sensor.AddObservation(rootRot);
        sensor.AddObservation(rootAB.velocity);
        sensor.AddObservation(rootAB.angularVelocity);

        for (var index = 1; index < jointNum; index++)
        {
            var jointAB = AgentABs[index];
            var jointT = AgentTransforms[index];

            sensor.AddObservation(rootInv * (jointT.position - root.position));
            sensor.AddObservation(rootInv * jointT.rotation);
            sensor.AddObservation(jointAB.velocity - rootAB.velocity);
            sensor.AddObservation(jointAB.angularVelocity - rootAB.angularVelocity);
        }

        // Time to next frame (1)
        // sensor.AddObservation(currentFrame + 1 - animationTime * 30f);

        // Target State (16*13) * 3 = 624
        for (var i = 0; i < targetStatesLength; ++i)
        {
            var targetFrame = Mathf.Min(currentFrame + FrameOffset[i], currentMotion.data.Count - 1);
            var targetPose = currentMotion.data[targetFrame];

            foreach (var joint in targetPose.joints)
            {
                sensor.AddObservation(joint.position);
                sensor.AddObservation(joint.rotation);
                sensor.AddObservation(joint.velocity);
                sensor.AddObservation(joint.angularVelocity);
            }
        }
    }


    public void AddTargetStateReward()
    {
        _episodeTime += Time.fixedDeltaTime;
        var animationTime = _initTime + _episodeTime;
        var curFrame = (int) (animationTime * fps);
        var targetPose = currentMotion.data[curFrame];
        if (_currentFrame != curFrame)
        {
        }

        _currentFrame = curFrame;

        _isTerminated = false;

        // 193 + 600 = 793

        // TODO: Recalculate reward function
        var root = AgentTransforms[0];
        var rootAB = AgentABs[0];
        var targetRoot = targetPose.joints[0];
        var rootParent = root.parent;
        var parentRot = rootParent.localRotation;
        var rootPos = rootParent.localPosition + parentRot * root.localPosition;
        var rootRot = parentRot * root.localRotation;
        var rootInv = Quaternion.Inverse(rootRot);

        var posReward = rootPos.y - targetRoot.position.y;
        posReward *= posReward;
        // var posReward = (rootPos - targetRoot.position).sqrMagnitude;

        var rotReward = Quaternion.Angle(rootRot, targetRoot.rotation) / 180 * Mathf.PI; // degrees
        rotReward *= rotReward;

        // var velReward = rootAB.velocity.y - targetRoot.velocity.y;
        // velReward *= velReward;
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
            if (index != 3 &&
                index != 6 &&
                index != 11 &&
                index != 15) continue;
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


        var com = Vector3.zero;
        var totalMass = 0f;
        foreach (var jointAB in AgentABs)
        {
            var mass = jointAB.mass;
            com += (jointAB.worldCenterOfMass - root.position) * mass;
            totalMass += mass;
        }

        com /= totalMass;
        com -= targetPose.centerOfMass;
        var comReward = com.sqrMagnitude;


#if UNITY_EDITOR
        // print($"Total reward: {posReward} + {rotReward} + {velReward} + {avReward}\n" +
        //       $"Joint reward : {totalJointPosReward} + {totalJointRotReward} + {totalJointVelReward}");
#endif

        posReward += totalJointPosReward / 40f;
        rotReward += totalJointRotReward / 5f;
        velReward += totalJointVelReward / 40f;
        avReward += totalJointAvReward / 10f;


        posReward = Mathf.Exp(posReward * -5);
        rotReward = Mathf.Exp(rotReward / -2);
        velReward = Mathf.Exp(velReward / -20);
        avReward = Mathf.Exp(avReward / -50);
        comReward = Mathf.Exp(comReward * -100);

#if UNITY_EDITOR
        print($"Total reward: {posReward} + {rotReward} + {velReward} + {avReward} + {comReward}\n");
#endif
        // var totalReward = (posReward + rotReward + velReward / 2 + avReward / 2) / 1.5f - 1f;
        var totalReward = (posReward + rotReward + velReward / 5 + avReward / 5 + comReward) / 1.7f - 1f;
// #if !UNITY_EDITOR
        if (posReward < 0.2 || rotReward < 0.2)
        {
            _isTerminated = true;
            totalReward = -1;
        }

// #endif
        _isRewarded = true;
        AddReward(totalReward);

        if (_currentFrame == currentMotion.data.Count - 1)
        {
            _earlyTerminationStack = EarlyTerminationMax;
            _isTerminated = true;
        }

        if (_isTerminated) EarlyTerminate();
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
}