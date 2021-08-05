using System;
using UnityEngine;

namespace DataProcessor
{
    [Serializable]
    public struct JointData
    {
        public int jointIdx;
        public int parentIdx;
        public int[] childrenIdx;
        public string jointName;
        public Vector3 position;
        public Quaternion rotation;
        public Quaternion inverseRotation;
        public Vector3 velocity;
        public Vector3 angularVelocity;

        public JointData(int idx)
        {
            parentIdx = -1;
            jointIdx = idx;
            jointName = default;
            childrenIdx = new int[] { };
            position = default;
            rotation = default;
            inverseRotation = default;
            velocity = default;
            angularVelocity = default;
        }
    }
}