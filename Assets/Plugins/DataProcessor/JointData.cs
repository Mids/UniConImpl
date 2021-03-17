using UnityEngine;

namespace DataProcessor
{
    public class JointData
    {
        public Vector3 Position;
        public Quaternion Rotation;
        public Vector3 Velocity;
        public Vector3 AngularVelocity;

        public void SetPositionAndRotation(Vector3 position, Quaternion rotation)
        {
            Position = position;
            Rotation = rotation;
        }

        public override string ToString()
        {
            return
                $"{Position.x}\t{Position.y}\t{Position.z}\t{Rotation.x}\t{Rotation.y}\t{Rotation.z}\t{Rotation.w}\t{Velocity.x}\t{Velocity.y}\t{Velocity.z}\t{AngularVelocity.x}\t{AngularVelocity.y}\t{AngularVelocity.z}";
        }
    }
}