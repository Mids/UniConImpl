using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

[RequireComponent(typeof(ArticulationBody))]
public class RagdollJoint : MonoBehaviour
{
    public int dofCount = 0;
    public Vector3 powerVector = new Vector3(500, 500, 500);
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

    public void ResetJoint(Quaternion rootRot, JointData data)
    {
        if (_isRoot)
        {
            _ab.TeleportRoot(transform.parent.position, data.rotation);
        }
        else
        {
            // var newRot = _initRootRotation * _initPose.joints[i].rotation;
            // var localRot = Quaternion.Inverse(_initRotations[i]) * newRot;
            // var euler = localRot.eulerAngles * Mathf.PI / 180f;
            // var x = euler.x > Mathf.PI ? euler.x - 2 * Mathf.PI : euler.x;
            // var y = euler.y > Mathf.PI ? euler.y - 2 * Mathf.PI : euler.y;
            // var z = euler.z > Mathf.PI ? euler.z - 2 * Mathf.PI : euler.z;

            // var newRot = rootRot * data.rotation;
            // var parentRot = rootRot * 

            _ab.ResetInertiaTensor();
            _ab.jointPosition = new ArticulationReducedSpace(0, 0, 0);
        }

        _ab.velocity = data.velocity;
        _ab.angularVelocity = data.angularVelocity;
    }

    private void AddRelativeTorque(Vector3 v)
    {
        _ab.AddRelativeTorque(Vector3.Scale(v, powerVector));
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

    private void Awake()
    {
        _ab = GetComponent<ArticulationBody>();
        _isRoot = _ab.isRoot;
        dofCount = _ab.dofCount;
    }
}