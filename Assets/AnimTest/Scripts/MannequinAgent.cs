using System.Collections;
using System.Collections.Generic;
using System.IO;
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

    public List<float> powerVector;
    private float[] _lastActionsArray;
    private float[] _lastActionsDiffArray;

    private Quaternion[] _initRotations;

#if UNITY_EDITOR
    public bool curl = false;
    public List<Vector2> curlPair = new List<Vector2>();

    private StreamWriter _sw;
#endif // UNITY_EDITOR

    // public AnimationPlayer RefModel;
    // private Transform RefTransform;
    // private List<Transform> RefTransforms;

    public override void Initialize()
    {
        base.Initialize();
        _trainingArea = GetComponentInParent<TrainingArea>();
#if UNITY_EDITOR
        _RefAP = GetComponentInChildren<AnimationPlayer>();
        _sw = new StreamWriter("actionOutput.txt");
#endif // UNITY_EDITOR

        // RefTransform = _trainingArea.mannequinRef.transform;
        // var refABs = RefTransform.GetComponentsInChildren<ArticulationBody>().ToList();
        // RefTransforms = refABs.Select(p => p.transform).ToList();
        // RefModel = _trainingArea.mannequinRef.GetComponent<AnimationPlayer>();

        AgentABs = AgentMesh.GetComponentsInChildren<ArticulationBody>().ToList();
        AgentTransforms = AgentABs.Select(p => p.transform).ToList();
        _initRotations = AgentTransforms.Select(p => p.rotation).ToArray();

        Physics.IgnoreCollision(AgentABs[1].GetComponent<Collider>(), AgentABs[2].GetComponent<Collider>());
        Physics.IgnoreCollision(AgentABs[2].GetComponent<Collider>(), AgentABs[3].GetComponent<Collider>());
        Physics.IgnoreCollision(AgentABs[4].GetComponent<Collider>(), AgentABs[5].GetComponent<Collider>());
        Physics.IgnoreCollision(AgentABs[5].GetComponent<Collider>(), AgentABs[6].GetComponent<Collider>());
        Physics.IgnoreCollision(AgentABs[1].GetComponent<Collider>(), AgentABs[7].GetComponent<Collider>());
        Physics.IgnoreCollision(AgentABs[4].GetComponent<Collider>(), AgentABs[7].GetComponent<Collider>());
        Physics.IgnoreCollision(AgentABs[7].GetComponent<Collider>(), AgentABs[8].GetComponent<Collider>());
        Physics.IgnoreCollision(AgentABs[8].GetComponent<Collider>(), AgentABs[9].GetComponent<Collider>());
        Physics.IgnoreCollision(AgentABs[9].GetComponent<Collider>(), AgentABs[10].GetComponent<Collider>());
        Physics.IgnoreCollision(AgentABs[10].GetComponent<Collider>(), AgentABs[11].GetComponent<Collider>());
        Physics.IgnoreCollision(AgentABs[8].GetComponent<Collider>(), AgentABs[12].GetComponent<Collider>());
        Physics.IgnoreCollision(AgentABs[8].GetComponent<Collider>(), AgentABs[13].GetComponent<Collider>());
        Physics.IgnoreCollision(AgentABs[13].GetComponent<Collider>(), AgentABs[14].GetComponent<Collider>());
        Physics.IgnoreCollision(AgentABs[14].GetComponent<Collider>(), AgentABs[15].GetComponent<Collider>());
    }

    public override void OnEpisodeBegin()
    {
        _isInitialized = false;
        _episodeTime = 0f;
        _earlyTerminationStack = 0;
        _currentFrame = -1;
        for (var i = 0; i < _lastActionsArray?.Length; i++) _lastActionsArray[i] = 0f;
        for (var i = 0; i < _lastActionsDiffArray?.Length; i++) _lastActionsDiffArray[i] = 0f;

#if UNITY_EDITOR
        _lastReward = 0;
#endif // UNITY_EDITOR

        var motionIndex = _trainingArea.GetRandomMotionIndex();
        currentMotion = _trainingArea.GetMotion(motionIndex);
#if UNITY_EDITOR
        _RefAP.SetMotion(currentMotion);
#endif // UNITY_EDITOR

        // TODO: RSI
        // _initFrame = Random.Range(0, _trainingArea.GetMotionTotalFrame(motionIndex));
        _initFrame = 0;
        _initTime = _initFrame / fps;
        _currentFrame = _initFrame;

        _initPose = currentMotion.data[_currentFrame];
        _initRootPosition = transform.position + new Vector3(0, 0.8f, 0);
        _initRootRotation = _initPose.joints[0].rotation;
        _initRootRotationInv = Quaternion.Inverse(_initRootRotation);

        ResetAgentPose();
        StartCoroutine(WaitForInit());
    }

    private IEnumerator WaitForInit()
    {
        yield return new WaitForFixedUpdate();
        _isInitialized = true;
    }

    private void ResetAgentPose()
    {
        AgentABs[0].TeleportRoot(_initRootPosition, _initRootRotation);
        AgentABs[0].velocity = _initPose.joints[0].velocity;
        AgentABs[0].angularVelocity = _initPose.joints[0].angularVelocity;

        for (int i = 1; i < AgentTransforms.Count; ++i)
        {
            AgentABs[i].ResetInertiaTensor();
            // AgentABs[i].jointPosition = new ArticulationReducedSpace(0, 0, 0);

            var newRot = _initRootRotation * _initPose.joints[i].rotation;
            var localRot = Quaternion.Inverse(_initRotations[i]) * newRot;
            var euler = localRot.eulerAngles * Mathf.PI / 180f;
            var x = euler.x > Mathf.PI ? euler.x - 2 * Mathf.PI : euler.x;
            var y = euler.y > Mathf.PI ? euler.y - 2 * Mathf.PI : euler.y;
            var z = euler.z > Mathf.PI ? euler.z - 2 * Mathf.PI : euler.z;
            AgentABs[i].jointPosition = new ArticulationReducedSpace(x, y, z);

            AgentABs[i].velocity = _initPose.joints[i].velocity;
            AgentABs[i].angularVelocity = _initPose.joints[i].angularVelocity;
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 208+624 = 832
        var currentFrame = _currentFrame;
        var jointNum = AgentABs.Count;
        var rootAB = AgentABs[0];
        var root = AgentTransforms[0];
        var rootRot = root.localRotation;
        var rootInv = Quaternion.Inverse(rootRot);

        // Current State (16 * 13) = 208
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
#if UNITY_EDITOR
        var currentPose = currentMotion.data[currentFrame];
        _RefAP.SetPose(currentPose);
#endif // UNITY_EDITOR


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

    public override void OnActionReceived(ActionBuffers actions)
    {
        var actionsArray = actions.ContinuousActions.Array;
        var forcePenalty = 0f;
        var actionsArrayLength = actionsArray.Length;
        _lastActionsArray ??= new float[actionsArrayLength];
        _lastActionsDiffArray ??= new float[actionsArrayLength];

        _episodeTime += Time.fixedDeltaTime;
        var animationTime = _initTime + _episodeTime;
        var curFrame = (int) (animationTime * fps);
        _currentFrame = curFrame;

#if UNITY_EDITOR
        var actionString = actionsArray.Aggregate("", (current, action) => current + (action + "\t"));
        _sw.WriteLine(actionString);
#endif // UNITY_EDITOR

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

#if UNITY_EDITOR
        if (curl)
            foreach (var kvp in curlPair)
                actionsArray[(int) kvp.x] = kvp.y;
#endif // UNITY_EDITOR

        // 3 * 10 + 1 * 4 = 34
        var actionIndex = 0;
        for (var i = 1; i < AgentABs.Count; ++i)
            if (AgentABs[i].dofCount == 1)
            {
                AgentABs[i].AddRelativeTorque(new Vector3(
                    0, 0, powerVector[i] * actionsArray[actionIndex]));

                actionIndex += 1;
            }
            else if (AgentABs[i].dofCount == 3)
            {
                AgentABs[i].AddRelativeTorque(new Vector3(
                    powerVector[i] * actionsArray[actionIndex],
                    powerVector[i] * actionsArray[actionIndex + 1],
                    powerVector[i] * actionsArray[actionIndex + 2]));
                actionIndex += 3;
            }


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
        var targetPose = currentMotion.data[_currentFrame];

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

        posReward += totalJointPosReward / 8f;
        rotReward += totalJointRotReward / 4f;
        velReward += totalJointVelReward / 4f;
        avReward += totalJointAvReward / 4f;


        posReward = Mathf.Exp(posReward * -4);
        rotReward = Mathf.Exp(rotReward / -4);
        velReward = Mathf.Exp(velReward / -10);
        avReward = Mathf.Exp(avReward / -20);
        comReward = Mathf.Exp(comReward * -200);

#if UNITY_EDITOR
        // print($"Total reward: {posReward} + {rotReward} + {velReward} + {avReward} + {comReward}\n");
#endif
        // var totalReward = (posReward + rotReward + velReward / 2 + avReward / 2) / 1.5f - 1f;
        var totalReward = (posReward + rotReward + velReward / 2 + avReward / 2 + comReward) / 4f - 0.1f;

        if (totalReward < 0f || AgentTransforms[12].position.y < 0.3f)
        {
            _isTerminated = true;
            totalReward = -1;
        }
        else
        {
            for (var index = 0; index < AgentTransforms.Count; index++)
            {
                if (index == 3 || index == 6 || AgentTransforms[index].position.y > 0.1f)
                    continue;

                _isTerminated = true;
                totalReward = -1;
                break;
            }
        }

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


    private void FixedUpdate()
    {
        if (!_isInitialized) return;

        RequestDecision();
    }

#if UNITY_EDITOR
    private void OnDestroy()
    {
        _sw.Close();
    }

#endif // UNITY_EDITOR
}