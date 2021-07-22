using System.Collections.Generic;
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
    private static Quaternion _rootInv;
    private static Vector3 _rootVelocity;
    private static Vector3 _rootAngularVelocity;
    private const int AvThreshold = 20;

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
        }

        _ab.velocity = rootRot * targetJointData.velocity;
        _ab.angularVelocity = Vector3.zero;
        // _ab.angularVelocity = rootRot * targetJointData.angularVelocity;
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
            var rootRot = parentRot * t.localRotation;

            _rootPos = parent.localPosition + parentRot * t.localPosition;
            _rootInv = Quaternion.Inverse(rootRot);
            _rootVelocity = _ab.velocity;
            _rootAngularVelocity = _ab.angularVelocity;

            sensor.AddObservation(_rootPos.y);
            sensor.AddObservation(rootRot);
            sensor.AddObservation(_rootInv * _rootVelocity);
            sensor.AddObservation(_rootAngularVelocity * Mathf.Deg2Rad);
        }
        else
        {
            var localRot = Quaternion.Inverse(_parentTransform.rotation) * t.rotation;
            var v = _ab.velocity - _rootVelocity;
            var av = _ab.angularVelocity - _rootAngularVelocity;
            var rc = LocalToReducedCoordinate(localRot);

            sensor.AddObservation(_rootInv * (t.position - _rootTransform.position));
            sensor.AddObservation(rc);
            sensor.AddObservation(_rootInv * v);
            sensor.AddObservation(av * Mathf.Deg2Rad);

            if (av.magnitude > AvThreshold) result = false;
        }

        return result;
    }

    public void AddTargetObservation(VectorSensor sensor, JointData target)
    {
        if (_isRoot)
        {
            sensor.AddObservation(target.position.y);
            sensor.AddObservation(target.rotation);
            sensor.AddObservation(target.velocity);
            sensor.AddObservation(target.angularVelocity * Mathf.Deg2Rad);
        }
        else
        {
            sensor.AddObservation(target.position);

            var localTargetRot = Quaternion.Inverse(_parent.targetJointData.rotation) * targetJointData.rotation;
            if (_parent._isRoot)
                localTargetRot = targetJointData.rotation;

            sensor.AddObservation(LocalToReducedCoordinate(localTargetRot));
            sensor.AddObservation(target.velocity);
            sensor.AddObservation(target.angularVelocity * Mathf.Deg2Rad);
        }
    }

    private void AddRelativeTorque(Vector3 v)
    {
        if (isManual)
        {
            SetPDTarget();
            return;
        }

        var exaggeratedVector = v;
        exaggeratedVector.Scale(powerVector);
        // v.Scale(powerVector / _ab.mass);
        // v *= 10 / _ab.mass;

        var localTorque = _ab.parentAnchorRotation * exaggeratedVector;
        var worldTorque = Quaternion.Inverse(_ab.transform.rotation) * localTorque;
        _parent._ab.AddTorque(-worldTorque);
        _ab.AddTorque(worldTorque);

        // var vel = _ab.jointVelocity;
        // if (dofCount == 3)
        // {
        //     vel[0] += v.y;
        //     vel[1] += v.z;
        //     vel[2] += v.x;
        // }
        // else if (dofCount == 1)
        // {
        //     vel[0] += v.z;
        // }
        //
        // _ab.jointVelocity = vel;
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
    }
}