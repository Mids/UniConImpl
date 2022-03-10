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

[RequireComponent(typeof(BehaviorParameters))]
public class MannequinAgent : Agent
{
    private TrainingArea _trainingArea;


    private float _episodeTime = 0f;
    private float _initTime = 0f;
    private int _initFrame = 0;
    private bool _isTerminated = false;
    private int _earlyTerminationStack = 0;
    private const int EarlyTerminationMax = 1;
    private int _currentFrame = -1;
    private bool _isInitialized = false;
    private Vector3 _lastCOM = Vector3.zero;
    private int _terminatedFrame = -1;
    private const float InitVel = 1f;
    public Vector3 targetRootPosition = Vector3.zero;
    public float velocity = 0f;
    public Vector3 direction = Vector3.forward;
    public float directionAngularVelocity = 0f;

    public float accel;
    // private int _episodeCnt = 0;

    private static readonly int[] FrameOffset = {1, 4, 16};
    private static readonly int MaxStepInEpisode = 1000;

    public MotionData currentMotion;
    private SkeletonData currentPose => currentMotion.data[_currentFrame];
    private SkeletonData _initPose;
    private Quaternion _initRotation;
    public GameObject AgentMesh;
    public float fps = 30f;

    public List<Transform> AgentTransforms;
    public List<ArticulationBody> AgentABs;
    public List<FootContact> Feet;
    public RagdollController ragdollController;

#if UNITY_EDITOR
    [Header("Debug")]
    public TextMeshPro cumulativeRewardText;

    public bool drawDebugLine;
    private float _lastReward = 0;
    public GameObject RefPrefab;
    public int RefCount;
    public int RefInterval;
    private readonly List<AnimationPlayer> _RefAP = new List<AnimationPlayer>();
#endif // UNITY_EDITOR

    #region ML-Agents

    public override void Initialize()
    {
        base.Initialize();
        _trainingArea = GetComponentInParent<TrainingArea>();
#if UNITY_EDITOR
        for (int i = 0; i < RefCount; ++i)
            _RefAP.Add(Instantiate(RefPrefab, Vector3.zero, Quaternion.identity, transform)
                .GetComponent<AnimationPlayer>());
#endif // UNITY_EDITOR

        ragdollController = GetComponent<RagdollController>();

        AgentABs = AgentMesh.GetComponentsInChildren<ArticulationBody>().ToList();
        AgentTransforms = AgentABs.Select(p => p.transform).ToList();
        Feet = AgentMesh.GetComponentsInChildren<FootContact>().ToList();
        _initRotation = AgentTransforms[0].rotation;
    }

    public override void OnEpisodeBegin()
    {
        _isInitialized = false;
        _episodeTime = 0f;
        _earlyTerminationStack = 0;
        // ++_episodeCnt;
#if UNITY_EDITOR
        // _episodeCnt = 5000 * ((int) InitVel + 2);
#endif // UNITY_EDITOR
        // velocity = Random.Range(InitVel, Mathf.Clamp(_episodeCnt / 5000f, InitVel, InitVel + 1f));
        velocity = 0f;
        direction = Vector3.forward;
        directionAngularVelocity = 0f;
        targetRootPosition = new Vector3(0, 0.74f, 0);

        var motionIndex = _trainingArea.GetRandomMotionIndex();
        if (_terminatedFrame > 0)
        {
            _initFrame = Mathf.Max(_terminatedFrame - 100, 0);
        }
        else
        {
            currentMotion = _trainingArea.GetMotion(motionIndex);
            _initFrame = Random.Range(0, _trainingArea.GetMotionTotalFrame(motionIndex) - MaxStepInEpisode);
        }

        _terminatedFrame = -1;

#if UNITY_EDITOR
        _lastReward = 0;
        for (int i = 0; i < RefCount; ++i)
        {
            _RefAP[i].SetMotion(currentMotion);
            _RefAP[i].JointTransforms[0].position = transform.position + targetRootPosition;
        }
#endif // UNITY_EDITOR

        // _initFrame = 0;
        _initTime = _initFrame / fps;
        _currentFrame = _initFrame;

        _initPose = currentPose;
        Feet.ForEach(p => p.isContact = false);
        _lastCOM = targetRootPosition + _initPose.centerOfMass;

        ResetAgentPose();
        StartCoroutine(WaitForInit());
    }

    public override void CollectObservations(VectorSensor sensor)
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
        var targetCOM = currentPose.joints[0].rotation * currentPose.centerOfMass + targetRootPosition;


        sensor.AddObservation(Feet[0].isContact);
        sensor.AddObservation(Feet[1].isContact);
        // sensor.AddObservation(com);
        // sensor.AddObservation(comDir * fps);
        // sensor.AddObservation(targetCOM);
        sensor.AddObservation(targetCOM - com);


        if (!ragdollController.OnCollectObservations(sensor))
            _isTerminated = true;

        foreach (var offset in FrameOffset)
        {
            var targetFrame = Mathf.Min(_currentFrame + offset, currentMotion.data.Count - 1);
            var targetPose = currentMotion.data[targetFrame];
            targetPose.joints = (JointData[]) targetPose.joints.Clone();
            var targetDirection = Quaternion.AngleAxis(offset * directionAngularVelocity, Vector3.up) * direction;

            targetPose.joints[0].position = GetTargetPositionFor(offset, targetRootPosition, direction);
            targetPose.joints[0].velocity = velocity * targetDirection;
            targetPose.joints[0].rotation = Quaternion.LookRotation(targetDirection) * targetPose.joints[0].rotation;
            targetPose.joints[0].angularVelocity += new Vector3(0, directionAngularVelocity, 0);

            ragdollController.AddTargetObservation(sensor, targetPose);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var forcePenalty = ragdollController.OnActionReceived(actions.ContinuousActions.Array);

        SetReward(0);
        AddTargetStateReward();
        // AddReward((0.5f - forcePenalty) / 100);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        for (var index = 0; index < actionsOut.ContinuousActions.Array.Length; index++)
            actionsOut.ContinuousActions.Array[index] = Random.Range(-1f, 1f);
    }

    #endregion

    private IEnumerator WaitForInit()
    {
        yield return new WaitForEndOfFrame();
        _episodeTime = 0f;
        ragdollController.ResetRagdoll(_initPose, targetRootPosition, velocity * direction);
        ragdollController.FreezeAll(false);
        RequestDecision();
        _isInitialized = true;
    }

    private void ResetAgentPose()
    {
        ragdollController.FreezeAll(true);
        ragdollController.ResetRagdoll(_initPose, targetRootPosition, velocity * direction);
    }


    private void AddTargetStateReward()
    {
        var targetPose = currentPose;
        targetPose.joints = (JointData[]) targetPose.joints.Clone();
        targetPose.joints[0].rotation =
            Quaternion.LookRotation(direction, Vector3.up) * targetPose.joints[0].rotation;

        _isTerminated = false;

        var root = AgentTransforms[0];
        var rootAB = AgentABs[0];
        var rootParent = root.parent;
        var parentRot = rootParent.rotation;
        var rootPos = rootParent.localPosition + parentRot * root.localPosition;
        var rootRot = parentRot * root.localRotation;
        var rootInv = Quaternion.Inverse(rootRot);

        var targetRoot = targetPose.joints[0];

        var projRoot = Vector3.ProjectOnPlane(rootRot * Vector3.forward, Vector3.up);
        var projTargetRoot = Vector3.ProjectOnPlane(targetRoot.rotation * Vector3.forward, Vector3.up);
        var projRootRot = Quaternion.FromToRotation(projRoot, projTargetRoot);

        // var posReward = rootPos.y - targetRoot.position.y;
        // posReward *= posReward;
        var posReward = (rootPos - targetRootPosition).magnitude;

        var rotReward = 1 - Mathf.Abs((rootInv * targetRoot.rotation).w);
        // rotReward *= rotReward;

        var velReward = (rootAB.velocity - velocity * direction).magnitude;

        var avReward = (rootAB.angularVelocity - targetRoot.angularVelocity).magnitude;

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
                index != 12 &&
                index != 15) continue;

            var jointT = AgentTransforms[index];
            var jointAB = AgentABs[index];
            var targetData = targetPose.joints[index];

            var posDiff = projRootRot * (jointT.position - root.position) - targetRoot.rotation * targetData.position;
            var jointPosReward = posDiff.magnitude;
            totalJointPosReward += jointPosReward;

            if (index == 3 || index == 6 || index == 12)
            {
                var rotDiff = Quaternion.Inverse(rootInv * jointT.rotation) * targetData.rotation;
                var jointRotReward = 1 - rotDiff.w;
                // jointRotReward *= jointRotReward;
                totalJointRotReward += jointRotReward;
            }

            var velDiff = projRootRot * (jointAB.velocity - rootAB.velocity) -
                          targetRoot.rotation * targetData.velocity;
            var jointVelReward = velDiff.magnitude;
            totalJointVelReward += jointVelReward;

            var avDiff = jointAB.angularVelocity - rootAB.angularVelocity - targetData.angularVelocity;
            var jointAvReward = avDiff.magnitude;
            totalJointAvReward += jointAvReward;
        }

        posReward += totalJointPosReward / 5f;
        rotReward += totalJointRotReward / 3f;
        velReward += totalJointVelReward / 5f;
        avReward += totalJointAvReward / 5f;
        var comVelReward = GetComVelReward();


        posReward = Mathf.Exp(posReward * -1f);
        rotReward = Mathf.Exp(rotReward * -2f);
        velReward = Mathf.Exp(velReward / -10f);
        avReward = Mathf.Exp(avReward / -20f);
        comVelReward = Mathf.Exp(comVelReward * -1f);

        var curHead = AgentTransforms[12].position - rootParent.parent.position;
        var targetHead = targetRootPosition + targetRoot.rotation * targetPose.joints[12].position;
        var distHead = (targetHead - curHead).magnitude;
        var distFactor = Mathf.Clamp(1.1f - 1.2f * distHead, 0f, 1f);

        var totalReward = distFactor / 5f;
        if (distFactor > 0.8f
            && Vector3.Dot(rootAB.velocity, velocity * direction) >= 0f
            && Vector3.Dot(root.up, direction) >= 0f)
            totalReward += (posReward + rotReward + velReward + comVelReward) / 5f;

#if UNITY_EDITOR
        var rootWorldPos = root.position;
        Debug.DrawLine(rootWorldPos, rootWorldPos + Vector3.ProjectOnPlane(
            Quaternion.Inverse(_initRotation) * rootRot * Vector3.forward,
            Vector3.up) * 10, Color.magenta);
        Debug.DrawLine(rootWorldPos, rootWorldPos + direction * 10, Color.cyan);
        if (distFactor > 0.8f
            && Vector3.Dot(rootAB.velocity, velocity * direction) >= 0f
            && Vector3.Dot(root.up, direction) >= 0f)
            print(
                $"{distFactor} / 2 + ({posReward} + {rotReward} + {velReward} + {comVelReward}) / 8f = {totalReward}");
        else
            print($"{distFactor}");
#endif // UNITY_EDITOR

        if (distHead > 1f || AgentTransforms[12].position.y < 0.3f)
        {
            _isTerminated = true;
            totalReward = -1f;
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

        AddReward(totalReward);

        if (_currentFrame == currentMotion.data.Count - 1 || _currentFrame - _initFrame >= MaxStepInEpisode)
        {
            _earlyTerminationStack = EarlyTerminationMax;
            _isTerminated = true;
            AddReward(1);
        }

        if (_isTerminated) EarlyTerminate();
    }

    private float GetComVelReward()
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

        {
            var comVel = comDir * fps;
            comVel.y = 0f;

            var curFrame = _currentFrame;
            var lastFrame = curFrame - 1;
            if (lastFrame < 0)
            {
                lastFrame = 0;
                curFrame = 1;
            }

            var curPose = currentMotion.data[curFrame];
            var lastPose = currentMotion.data[lastFrame];
            var curCom = curPose.joints[0].position + curPose.joints[0].rotation * curPose.centerOfMass;
            var lastCom = lastPose.joints[0].position + lastPose.joints[0].rotation * lastPose.centerOfMass;
            var targetComVel = curCom - lastCom;
            targetComVel += velocity / fps * Vector3.forward;
            targetComVel *= fps;
            targetComVel.y = 0;


            // TODO: com vel reward is wrong
            targetComVel = velocity * direction;

            var diff = comVel - targetComVel;

            return diff.magnitude;
        }
    }

    private Vector3 GetTargetPositionFor(int offset, Vector3 current, Vector3 direction)
    {
        if (offset == 0)
            return current;

        return GetTargetPositionFor(offset - 1,
            current + velocity / fps * direction,
            Quaternion.AngleAxis(directionAngularVelocity, Vector3.up) * direction
        );
    }

    private static float Sign(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        return (p1.x - p3.x) * (p2.z - p3.z) - (p2.x - p3.x) * (p1.z - p3.z);
    }

    private static bool PointInQuadrangle(Vector3 pt, Vector3 v1, Vector3 v2, Vector3 v3, Vector3 v4)
    {
        return PointInTriangle(pt, v1, v2, v3) ||
               PointInTriangle(pt, v1, v2, v4) ||
               PointInTriangle(pt, v1, v3, v4) ||
               PointInTriangle(pt, v2, v3, v4);
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


    private void FixedUpdate()
    {
        if (!_isInitialized) return;

        _episodeTime += Time.fixedDeltaTime;
        var animationTime = _initTime + _episodeTime;
        var curFrame = (int) (animationTime * fps);

        if (curFrame != _currentFrame)
        {
            _currentFrame = curFrame;

// #if UNITY_EDITOR
//             if (Input.GetKey(KeyCode.UpArrow))
//                 velocity = 2f;
//             else
//                 velocity = 0f;
//
//             if (Input.GetKey(KeyCode.LeftArrow))
//                 directionAngularVelocity = -1f;
//             else if (Input.GetKey(KeyCode.RightArrow))
//                 directionAngularVelocity = 1f;
//             else
//                 directionAngularVelocity = 0f;
// #else
            var elapsedFrame = _currentFrame - _initFrame;
            if (elapsedFrame == 200)
                velocity = 2f;
            else if (elapsedFrame == 400)
                directionAngularVelocity = Random.Range(-1, 2);
            else if (elapsedFrame % 200 == 0)
            {
                velocity = Random.Range(0, 2) * 2;
                directionAngularVelocity = Random.Range(-1, 2);
            }

            // #endif
            RequestDecision();
        }

        {
            var root = AgentTransforms[0];
            var rootAB = AgentABs[0];
            var rootParent = root.parent;
            var parentRot = rootParent.rotation;
            var rootPos = rootParent.localPosition + parentRot * root.localPosition;
            direction = Quaternion.AngleAxis(directionAngularVelocity, Vector3.up) * direction;
            var nextPos = rootPos + velocity / fps * direction;
            nextPos.y = targetRootPosition.y;
            targetRootPosition = nextPos;
        }
        ragdollController.ApplyTorque();
    }


    #region DEBUG

#if UNITY_EDITOR

    private void Update()
    {
        var root = AgentTransforms[0];
        var rootAB = AgentABs[0];
        var rootParent = root.parent;
        var parentRot = rootParent.rotation;

        for (int i = 0; i < RefCount; ++i)
        {
            var offset = i * RefInterval;
            var targetFrame = Mathf.Clamp(_currentFrame + offset, 0, _initFrame + MaxStepInEpisode);
            var targetDirection = Quaternion.AngleAxis(offset * directionAngularVelocity, Vector3.up) * direction;
            offset = targetFrame - _currentFrame;
            _RefAP[i].SetPose(currentMotion.data[targetFrame]);
            _RefAP[i].JointTransforms[0].position = rootParent.parent.position +
                                                    GetTargetPositionFor(offset, targetRootPosition, direction);
            _RefAP[i].JointTransforms[0].rotation =
                Quaternion.LookRotation(targetDirection) * _RefAP[i].JointTransforms[0].rotation;
        }

        ShowReward();

        // if (drawDebugLine) DrawCOMAndFootContact();
    }

    private void ShowReward()
    {
        var delta = GetCumulativeReward() - _lastReward;
        if (delta == 0f) return;

        cumulativeRewardText.text = delta.ToString("0.00");
        _lastReward = GetCumulativeReward();
    }

    private void DrawCOMAndFootContact()
    {
        var projectedCom = _lastCOM;
        projectedCom.y = 0;
        var lFootPos = Feet[0].transform.position;
        lFootPos.y = 0;
        var rFootPos = Feet[1].transform.position;
        rFootPos.y = 0;
        var lToePos = lFootPos + Feet[0].transform.rotation * new Vector3(0, 0.2f, 0);
        lToePos.y = 0;
        var rToePos = rFootPos + Feet[1].transform.rotation * new Vector3(0, 0.2f, 0);
        rToePos.y = 0;

        if (PointInQuadrangle(projectedCom, lFootPos, lToePos, rFootPos, rToePos))
        {
            Debug.DrawLine(projectedCom, lFootPos, Color.blue);
            Debug.DrawLine(projectedCom, rFootPos, Color.blue);
            Debug.DrawLine(projectedCom, lToePos, Color.blue);
            Debug.DrawLine(projectedCom, rToePos, Color.blue);
        }
        else
        {
            Debug.DrawLine(projectedCom, lFootPos, Color.magenta);
            Debug.DrawLine(projectedCom, rFootPos, Color.magenta);
            Debug.DrawLine(projectedCom, lToePos, Color.magenta);
            Debug.DrawLine(projectedCom, rToePos, Color.magenta);
        }

        Debug.DrawLine(lFootPos, lToePos, Feet[0].isContact ? Color.cyan : Color.red, 0f, false);
        Debug.DrawLine(rFootPos, rToePos, Feet[1].isContact ? Color.cyan : Color.red, 0f, false);
    }
#endif // UNITY_EDITOR

    #endregion
}