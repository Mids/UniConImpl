using System.Collections.Generic;
using DataProcessor;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.Assertions;

[RequireComponent(typeof(ArticulationBody))]
public class RagdollJoint : MonoBehaviour
{
    public int dofCount = 0;
    public Vector3 powerVector = new Vector3(700, 700, 700);
    public JointData targetJointData;
    private ArticulationBody _ab;
    private RagdollJoint _root;
    private Transform _rootTransform;
    private RagdollJoint _parent;
    private Transform _parentTransform;
    private readonly List<RagdollJoint> _children = new List<RagdollJoint>();

    private bool _isRoot = false;
    private static Vector3 _rootPos;
    private static Quaternion _rootRot;
    private static Quaternion _rootInv;
    private static Vector3 _rootVelocity;
    private static Vector3 _rootAngularVelocity;
    private const int AvThreshold = 20;

    private Vector3 _jointPosObs;
    private Vector3 _jointRotObs;
    private Vector3 _jointVelObs;
    private Vector3 _jointAVObs;

    public float stiffness;
    public float damping;

    public bool isManual = false;
    public Vector3 manualPDTarget = Vector3.zero;

    public void SetRootAndParent(RagdollJoint root, RagdollJoint parent)
    {
        _root = root;
        _rootTransform = root.transform;
        _parent = parent;
        _parent._children.Add(this);
        _parentTransform = _parent.transform;

        var c = GetComponent<Collider>();
        var pc = _parent.GetComponent<Collider>();
        if (c != default && pc != default)
            Physics.IgnoreCollision(c, pc);
    }

    public void SetTargetJoint(JointData data)
    {
        targetJointData = data;
    }

    public void ResetJoint(Quaternion rootRot)
    {
        if (_isRoot)
        {
            _ab.TeleportRoot(transform.parent.parent.position + targetJointData.position, targetJointData.rotation);

            _ab.velocity = targetJointData.velocity;
            // _ab.angularVelocity = Vector3.zero;
            _ab.angularVelocity = targetJointData.angularVelocity;
        }
        else
        {
            var localTargetRot = Quaternion.Inverse(_parent.targetJointData.rotation) * targetJointData.rotation;
            if (_parent._isRoot)
                localTargetRot = targetJointData.rotation;

            var rc = LocalToReducedCoordinate(localTargetRot);

            if (dofCount == 3)
            {
                _ab.jointPosition = new ArticulationReducedSpace(rc.x, rc.y, rc.z);
                _ab.jointVelocity = new ArticulationReducedSpace(0, 0, 0);
            }
            else if (dofCount == 1)
            {
                _ab.jointPosition = new ArticulationReducedSpace(rc.z);
                _ab.jointVelocity = new ArticulationReducedSpace(0);
            }

            _ab.velocity = rootRot * targetJointData.velocity;
            // _ab.angularVelocity = Vector3.zero;
            _ab.angularVelocity = rootRot * targetJointData.angularVelocity;
        }
    }

    private Vector3 LocalToReducedCoordinate(Quaternion local)
    {
        Vector3 result;

        var localDiff = (Quaternion.Inverse(_ab.parentAnchorRotation) * local).eulerAngles;

        result.x = Mathf.DeltaAngle(0, localDiff.x);
        result.y = Mathf.DeltaAngle(0, localDiff.y);
        result.z = Mathf.DeltaAngle(0, localDiff.z);
        result *= Mathf.Deg2Rad;

        return result;
    }

    public void Freeze(bool b)
    {
        _ab.immovable = b;
    }

    public bool OnCollectObservations(VectorSensor sensor)
    {
        var result = true;

        var t = transform;
        if (_isRoot)
        {
            var parent = t.parent;
            var parentRot = parent.rotation;

            _rootPos = parent.localPosition + parentRot * t.localPosition;
            _rootRot = parentRot * t.localRotation;
            _rootInv = Quaternion.Inverse(_rootRot);
            _rootVelocity = _ab.velocity;
            _rootAngularVelocity = _ab.angularVelocity;

            sensor.AddObservation(_rootPos);
            sensor.AddObservation(_rootPos.y);
            sensor.AddObservation(_rootRot);
            sensor.AddObservation(_rootInv * _rootVelocity);
            sensor.AddObservation(_rootAngularVelocity * Mathf.Deg2Rad);
        }
        else
        {
            var localRot = Quaternion.Inverse(_parentTransform.rotation) * t.rotation;
            var v = _ab.velocity - _rootVelocity;
            var av = _ab.angularVelocity - _rootAngularVelocity;

            _jointPosObs = _rootInv * (t.position - _rootTransform.position);
            _jointRotObs = LocalToReducedCoordinate(localRot);
            _jointVelObs = _rootInv * v;
            _jointAVObs = av * Mathf.Deg2Rad;

            sensor.AddObservation(_jointPosObs);
            if (dofCount == 3)
                sensor.AddObservation(_jointRotObs);
            else if (dofCount == 1)
                sensor.AddObservation(_jointRotObs.z);
            sensor.AddObservation(_jointVelObs);
            sensor.AddObservation(_jointAVObs);

            if (av.magnitude > AvThreshold) result = false;
        }

        return result;
    }

    public void AddTargetObservation(VectorSensor sensor, JointData target)
    {
        if (_isRoot)
        {
            sensor.AddObservation(target.position - _rootPos);
            // sensor.AddObservation(target.position.y - _rootPos.y);
            sensor.AddObservation(_rootInv * target.rotation);
            sensor.AddObservation(target.velocity - _rootInv * _rootVelocity);
            sensor.AddObservation((target.angularVelocity - _rootAngularVelocity) * Mathf.Deg2Rad);
        }
        else
        {
            sensor.AddObservation(target.position - _jointPosObs);

            var localTargetRot = Quaternion.Inverse(_parent.targetJointData.rotation) * targetJointData.rotation;
            if (_parent._isRoot)
                localTargetRot = targetJointData.rotation;
            var rc = LocalToReducedCoordinate(localTargetRot);

            if (dofCount == 3)
                sensor.AddObservation(rc - _jointRotObs);
            else if (dofCount == 1)
                sensor.AddObservation(rc.z - _jointRotObs.z);

            sensor.AddObservation(target.velocity - _jointVelObs);
            sensor.AddObservation((target.angularVelocity - _jointAVObs) * Mathf.Deg2Rad);
        }
    }

    private void AddRelativeTorque(Vector3 v)
    {
        if (isManual)
        {
            SetPDTarget();
            return;
        }

        // var exaggeratedVector = v;
        // exaggeratedVector.Scale(powerVector);
        //
        // var localTorque = _ab.parentAnchorRotation * exaggeratedVector;
        // var worldTorque = Quaternion.Inverse(_ab.transform.rotation) * localTorque;
        // _parent._ab.AddTorque(-worldTorque);
        // _ab.AddTorque(worldTorque);

        if (_ab.swingYLock == ArticulationDofLock.LimitedMotion)
            _ab.yDrive = SetTarget(_ab.yDrive, v.y);
        if (_ab.swingZLock == ArticulationDofLock.LimitedMotion)
            _ab.zDrive = SetTarget(_ab.zDrive, v.z);
        if (_ab.twistLock == ArticulationDofLock.LimitedMotion)
            _ab.xDrive = SetTarget(_ab.xDrive, v.x);
    }

    public void AddRelativeTorque(float x, float y, float z)
    {
        Assert.AreEqual(_ab.dofCount, 3);
        AddRelativeTorque(new Vector3(x, y, z));
    }

    public void AddRelativeTorque(float z)
    {
        Assert.AreEqual(_ab.dofCount, 1);
        AddRelativeTorque(new Vector3(0, 0, z));
    }

    public void SetPDTarget()
    {
        if (_isRoot) return;

        var localTargetRot = Quaternion.Inverse(_parent.targetJointData.rotation) * targetJointData.rotation;
        if (_parent._isRoot)
            localTargetRot = targetJointData.rotation;

        var localDiff = (Quaternion.Inverse(_ab.parentAnchorRotation) * localTargetRot).eulerAngles;

        if (isManual)
            localDiff = manualPDTarget;
        else
            manualPDTarget = localDiff;

        var x = Mathf.DeltaAngle(0, localDiff.x); // * Mathf.PI / 180f;
        var y = Mathf.DeltaAngle(0, localDiff.y); // * Mathf.PI / 180f;
        var z = Mathf.DeltaAngle(0, localDiff.z); // * Mathf.PI / 180f;

        if (_ab.twistLock != ArticulationDofLock.LockedMotion)
        {
            var drive = _ab.xDrive;
            drive.target = x;
            // drive.lowerLimit = -180f;
            // drive.upperLimit = 180f;
            drive.stiffness = 500;
            drive.damping = 10;
            _ab.xDrive = drive;
        }

        if (_ab.swingYLock != ArticulationDofLock.LockedMotion)
        {
            var drive = _ab.yDrive;
            drive.target = y;
            // drive.lowerLimit = -180f;
            // drive.upperLimit = 180f;
            drive.stiffness = 500;
            drive.damping = 10;
            _ab.yDrive = drive;
        }

        if (_ab.swingZLock != ArticulationDofLock.LockedMotion)
        {
            var drive = _ab.zDrive;
            drive.target = z;
            // drive.lowerLimit = -180f;
            // drive.upperLimit = 180f;
            drive.stiffness = 500;
            drive.damping = 10;
            _ab.zDrive = drive;
        }
    }

    private ArticulationDrive SetTarget(ArticulationDrive drive, float t)
    {
        var lower = drive.lowerLimit;
        var upper = drive.upperLimit;
        var range = upper - lower;
        var target = lower + (t + 1) / 2 * range;
        drive.target = target;
        drive.stiffness = stiffness;
        drive.damping = damping;
        return drive;
    }

    private void Awake()
    {
        _ab = GetComponent<ArticulationBody>();
        _isRoot = _ab.isRoot;
        dofCount = _ab.dofCount;
        _ab.solverIterations = 50;
        _ab.solverVelocityIterations = 50;
    }
}