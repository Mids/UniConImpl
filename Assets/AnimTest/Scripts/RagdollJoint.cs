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
    private Quaternion tPoseRotation;
    private Quaternion tPoseLocalRotation;


    public void SetRootAndParent(RagdollJoint root, RagdollJoint parent)
    {
        _root = root;
        _parent = parent;
        _parent._children.Add(this);

        var c = GetComponent<Collider>();
        var pc = _parent.GetComponent<Collider>();
        if (c != default && pc != default)
            Physics.IgnoreCollision(c, pc);

        tPoseRotation = transform.rotation;
        tPoseLocalRotation = Quaternion.Inverse(_parent.transform.rotation) * tPoseRotation;
    }

    public void SetTargetJoint(JointData data)
    {
        targetJointData = data;
    }

    public void ResetJoint(Quaternion rootRot)
    {
        if (_isRoot)
        {
            _ab.TeleportRoot(transform.parent.position, targetJointData.rotation);
        }
        else
        {
            // var newRot = _initRootRotation * _initPose.joints[i].rotation;
            // var localRot = Quaternion.Inverse(_initRotations[i]) * newRot;
            // var euler = localRot.eulerAngles * Mathf.PI / 180f;

            var newRot = rootRot * targetJointData.rotation;
            var parentRot = rootRot * _parent.targetJointData.rotation;

            var localTargetRot = Quaternion.Inverse(_parent.targetJointData.rotation) * targetJointData.rotation;
            if (_parent._isRoot)
                localTargetRot = targetJointData.rotation;

            var localDiff = Quaternion.Inverse(tPoseLocalRotation) * localTargetRot;
            var euler = localDiff.eulerAngles * Mathf.PI / 180f;
            var x = euler.x > Mathf.PI ? euler.x - 2 * Mathf.PI : euler.x;
            var y = euler.y > Mathf.PI ? euler.y - 2 * Mathf.PI : euler.y;
            var z = euler.z > Mathf.PI ? euler.z - 2 * Mathf.PI : euler.z;

            if (dofCount == 3)
                _ab.jointPosition = new ArticulationReducedSpace(x, y, z);
            else if (dofCount == 1)
                _ab.jointPosition = new ArticulationReducedSpace(z);
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
        _ab.AddRelativeTorque(Vector3.Scale(v, powerVector));
    }

    public void AddRelativeTorque(float x, float y, float z)
    {
        Assert.AreEqual(_ab.dofCount, 3);
        // AddRelativeTorque(new Vector3(x, y, z));

        var xDrive = _ab.xDrive;
        xDrive.target = x;
        _ab.xDrive = xDrive;

        var yDrive = _ab.yDrive;
        yDrive.target = y;
        _ab.yDrive = yDrive;

        var zDrive = _ab.zDrive;
        zDrive.target = z;
        _ab.zDrive = zDrive;
    }

    public void AddRelativeTorque(float z)
    {
        Assert.AreEqual(_ab.dofCount, 1);
        // AddRelativeTorque(new Vector3(0, 0, z));

        var zDrive = _ab.zDrive;
        zDrive.target = z;
        _ab.zDrive = zDrive;
    }

    private void Awake()
    {
        _ab = GetComponent<ArticulationBody>();
        _isRoot = _ab.isRoot;
        dofCount = _ab.dofCount;
    }
}