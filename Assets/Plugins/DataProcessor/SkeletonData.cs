using System;
using UnityEngine;

[Serializable]
public struct SkeletonData
{
    public int frameNumber;
    public Vector3 centerOfMass;
    public JointData[] joints;

    public SkeletonData(int nubmerOfJoints)
    {
        frameNumber = -1;
        centerOfMass = default;
        joints = new JointData[nubmerOfJoints];
    }
}