using System;
using UnityEngine;
using UnityEngine.Assertions;

namespace DataProcessor
{
    public class SkeletonData
    {
        public JointData Root;
        public JointData LeftFoot;
        public JointData RightFoot;
        public JointData LeftHand;
        public JointData RightHand;

        public static readonly HumanBodyBones[] allBones =
        {
            HumanBodyBones.Hips,
            HumanBodyBones.LeftFoot,
            HumanBodyBones.RightFoot,
            HumanBodyBones.LeftHand,
            HumanBodyBones.RightHand
        };

        public SkeletonData()
        {
        }

        public SkeletonData(string line)
        {
            var data = Array.ConvertAll(line.Split('\t'), float.Parse);

            for (var boneIndex = 0; boneIndex < allBones.Length; boneIndex++)
            {
                var bone = allBones[boneIndex];
                var baseI = boneIndex * 13;

                this[bone] = new JointData
                {
                    Position = {x = data[baseI + 0], y = data[baseI + 1], z = data[baseI + 2]},
                    Rotation = {x = data[baseI + 3], y = data[baseI + 4], z = data[baseI + 5], w = data[baseI + 6]},
                    Velocity = {x = data[baseI + 7], y = data[baseI + 8], z = data[baseI + 9]},
                    AngularVelocity = {x = data[baseI + 10], y = data[baseI + 11], z = data[baseI + 12]}
                };
            }
        }

        public JointData this[HumanBodyBones bone]
        {
            get
            {
                switch (bone)
                {
                    case HumanBodyBones.Hips:
                        return Root;
                    case HumanBodyBones.LeftFoot:
                        return LeftFoot;
                    case HumanBodyBones.RightFoot:
                        return RightFoot;
                    case HumanBodyBones.LeftHand:
                        return LeftHand;
                    case HumanBodyBones.RightHand:
                        return RightHand;
                }

                Assert.IsTrue(false, "Undefined Error!");
                return Root;
            }
            set
            {
                switch (bone)
                {
                    case HumanBodyBones.Hips:
                        Root = value;
                        return;
                    case HumanBodyBones.LeftFoot:
                        LeftFoot = value;
                        return;
                    case HumanBodyBones.RightFoot:
                        RightFoot = value;
                        return;
                    case HumanBodyBones.LeftHand:
                        LeftHand = value;
                        return;
                    case HumanBodyBones.RightHand:
                        RightHand = value;
                        return;
                }

                Assert.IsTrue(false, "Undefined Error!");
            }
        }

        public override string ToString()
        {
            return $"{Root}\t{LeftFoot}\t{RightFoot}\t{LeftHand}\t{RightHand}";
        }
    }
}