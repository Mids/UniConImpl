using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public enum RagdollJoint
{
    Pelvis,
    LeftHips, // X, Y
    LeftKnee, // X
    RightHips, // X, Y
    RightKnee, // X
    LeftArm, // X, Y
    LeftElbow, // X
    RightArm, // X, Y
    RightElbow, // X
    MiddleSpine, // X, Y
    Head, // X, Y
}

public class RagdollData
{
    public string Name;
    public Transform RefTransform;
    public Transform AgentTransform;
    public CharacterJoint CharacterJointComp;
    public Rigidbody RigidbodyComp;

    public RagdollData(string name)
    {
        Name = name;
    }

    public Vector3 GetPosition()
    {
        return AgentTransform.localPosition;
    }

    public Quaternion GetRotation()
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
        new Dictionary<RagdollJoint, RagdollData>
        {
            {RagdollJoint.Pelvis, new RagdollData("Hips")},
            {RagdollJoint.LeftHips, new RagdollData("LeftUpLeg")},
            {RagdollJoint.LeftKnee, new RagdollData("LeftLeg")},
            {RagdollJoint.RightHips, new RagdollData("RightUpLeg")},
            {RagdollJoint.RightKnee, new RagdollData("RightLeg")},
            {RagdollJoint.LeftArm, new RagdollData("LeftArm")},
            {RagdollJoint.LeftElbow, new RagdollData("LeftForeArm")},
            {RagdollJoint.RightArm, new RagdollData("RightArm")},
            {RagdollJoint.RightElbow, new RagdollData("RightForeArm")},
            {RagdollJoint.MiddleSpine, new RagdollData("Spine1")},
            {RagdollJoint.Head, new RagdollData("Head")},
        };
    
    // Start is called before the first frame update
    private void Start()
    {
        var ragdollDatas = RagdollDataDict.Values.ToArray();

        var keys = RagdollDataDict.Keys;

        var transforms = GetComponentsInChildren<Transform>();

        foreach (var t in transforms)
        {
            var data = ragdollDatas.FirstOrDefault(p => t.name.Contains(p.Name));
            if(data == default) continue;
            
            data.AgentTransform = t;

            var joint = t.GetComponent<CharacterJoint>();
            if (joint != default)
                data.CharacterJointComp = joint;

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