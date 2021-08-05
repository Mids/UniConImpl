using System;
using UnityEngine;

namespace DataProcessor
{
    [Serializable]
    public struct SkeletonData
    {
        public int frameNumber;
        public Vector3 centerOfMass;
        public Vector3 centerOfMassVelocity;
        public JointData[] joints;

        public SkeletonData(int nubmerOfJoints)
        {
            frameNumber = -1;
            centerOfMass = default;
            centerOfMassVelocity = default;
            joints = new JointData[nubmerOfJoints];
        }
    }
}