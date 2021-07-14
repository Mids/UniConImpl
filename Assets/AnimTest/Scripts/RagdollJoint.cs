using System.Collections.Generic;
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
    private RagdollJoint _parent;
    private readonly List<RagdollJoint> _children = new List<RagdollJoint>();
    private bool _isRoot = false;

    public float stiffness;
    public float damping;

    public bool isManual = false;
    public Vector3 manualPDTarget = Vector3.zero;

    public void SetRootAndParent(RagdollJoint root, RagdollJoint parent)
    {
        _root = root;
        _parent = parent;
        _parent._children.Add(this);

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

            var localDiff = (Quaternion.Inverse(_ab.parentAnchorRotation) * localTargetRot).eulerAngles;

            var x = Mathf.DeltaAngle(0, localDiff.x) * Mathf.PI / 180f;
            var y = Mathf.DeltaAngle(0, localDiff.y) * Mathf.PI / 180f;
            var z = Mathf.DeltaAngle(0, localDiff.z) * Mathf.PI / 180f;

            if (dofCount == 3)
            {
                _ab.jointPosition = new ArticulationReducedSpace(x, y, z);
                _ab.jointVelocity = new ArticulationReducedSpace(0, 0, 0);
            }
            else if (dofCount == 1)
            {
                _ab.jointPosition = new ArticulationReducedSpace(z);
                _ab.jointVelocity = new ArticulationReducedSpace(0);
            }
        }

        _ab.velocity = rootRot * targetJointData.velocity;
        _ab.angularVelocity = rootRot * targetJointData.angularVelocity;
    }

    public void Freeze(bool b)
    {
        _ab.immovable = b;
    }

    private void AddRelativeTorque(Vector3 v)
    {
        if (isManual)
        {
            SetPDTarget();
            return;
        }

        var exaggeratedVector = new Vector3(
            v.x * v.x * v.x,
            v.y * v.y * v.y,
            v.z * v.z * v.z);
        exaggeratedVector.Scale(powerVector);

        var localTorque1 = _ab.parentAnchorRotation * exaggeratedVector;
        var worldTorque1 = Quaternion.Inverse(_ab.transform.rotation) * localTorque1;
        _parent._ab.AddTorque(-worldTorque1);
        _ab.AddTorque(worldTorque1);
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