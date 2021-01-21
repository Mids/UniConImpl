using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public enum RagdollJoint
{
    Pelvis,
    LeftHips, // X, Y
    LeftKnee, // X
    LeftFoot,
    RightHips, // X, Y
    RightKnee, // X
    RightFoot,
    LeftArm, // X, Y
    LeftElbow, // X
    LeftHand,
    RightArm, // X, Y
    RightElbow, // X
    RightHand,
    MiddleSpine, // X, Y
    Head, // X, Y
}

public class RagdollData
{
    public static readonly Dictionary<RagdollJoint, string> RagdollJointNames = new Dictionary<RagdollJoint, string>()
    {
        {RagdollJoint.Pelvis, "Hips"},
        {RagdollJoint.LeftHips, "LeftUpLeg"},
        {RagdollJoint.LeftKnee, "LeftLeg"},
        {RagdollJoint.LeftFoot, "LeftToeBase"},
        {RagdollJoint.RightHips, "RightUpLeg"},
        {RagdollJoint.RightKnee, "RightLeg"},
        {RagdollJoint.RightFoot, "RightToeBase"},
        {RagdollJoint.LeftArm, "LeftArm"},
        {RagdollJoint.LeftElbow, "LeftForeArm"},
        {RagdollJoint.LeftHand, "LeftHand"},
        {RagdollJoint.RightArm, "RightArm"},
        {RagdollJoint.RightElbow, "RightForeArm"},
        {RagdollJoint.RightHand, "RightHand"},
        {RagdollJoint.MiddleSpine, "Spine1"},
        {RagdollJoint.Head, "Head"},
    };
    
    public string Name;
    public Transform AgentTransform;
    public Rigidbody RigidbodyComp;

    public RagdollData(string name)
    {
        Name = name;
    }

    public Vector3 GetPosition()
    {
        return AgentTransform.position;
    }

    public Quaternion GetRotation()
    {
        return AgentTransform.rotation;
    }

    public Vector3 GetLocalPosition()
    {
        return AgentTransform.localPosition;
    }

    public Quaternion GetLocalRotation()
    {
        return AgentTransform.localRotation;
    }

    public Vector3 GetVelocity()
    {
        return RigidbodyComp.velocity;
    }

    public Vector3 GetAngularVelocity()
    {
        return RigidbodyComp.angularVelocity;
    }
}

public class RagdollController : MonoBehaviour
{
    public readonly Dictionary<RagdollJoint, RagdollData> RagdollDataDict =
        new Dictionary<RagdollJoint, RagdollData>();

    // Start is called before the first frame update
    private void Start()
    {
        foreach (var kvp in RagdollData.RagdollJointNames) 
            RagdollDataDict[kvp.Key] = new RagdollData(kvp.Value);

        var ragdollDatas = RagdollDataDict.Values.ToArray();

        var transforms = GetComponentsInChildren<Transform>();

        foreach (var t in transforms)
        {
            var data = ragdollDatas.FirstOrDefault(p => t.name.Contains(p.Name));
            if (data == default) continue;

            data.AgentTransform = t;

            var rigidbodyComp = t.GetComponent<Rigidbody>();
            if (rigidbodyComp != default)
                data.RigidbodyComp = rigidbodyComp;
        }
    }

    public void AddTorque(RagdollJoint joint, float x, float y, float z)
    {
        AddTorque(joint, new Vector3(x, y, z));
    }

    public void AddTorque(RagdollJoint joint, Vector3 force)
    {
        if (!RagdollDataDict.ContainsKey(joint)) return;

        RagdollDataDict[joint].RigidbodyComp.AddTorque(force);
    }
}