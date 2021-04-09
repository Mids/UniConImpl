using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

namespace DataProcessor
{
    public class SkeletonData
    {
        public JointData Root;
        
        public SkeletonData()
        {
        }

        public string[] Joints;
        public readonly Dictionary<string, JointData> JointDict = new Dictionary<string, JointData>();

        public void InitDict(string line)
        {
            Joints = line.Split('\t');
        }

        public void InitData(string line)
        {
            Assert.IsNotNull(Joints, "InitDict First");
        
            var data = Array.ConvertAll(line.Split('\t'), float.Parse);

            Root = new JointData
            {
                Position = {x = data[0], y = data[1], z = data[2]},
                Rotation = {x = data[3], y = data[4], z = data[5], w = data[6]},
                Velocity = {x = data[7], y = data[8], z = data[9]},
                AngularVelocity = {x = data[10], y = data[11], z = data[12]}
            };

            for (var boneIndex = 0; boneIndex < Joints.Length; boneIndex++)
            {
                var bone = Joints[boneIndex];
                var baseI = boneIndex * 13 + 13;

                JointDict[bone] = new JointData
                {
                    Position = {x = data[baseI + 0], y = data[baseI + 1], z = data[baseI + 2]},
                    Rotation = {x = data[baseI + 3], y = data[baseI + 4], z = data[baseI + 5], w = data[baseI + 6]},
                    Velocity = {x = data[baseI + 7], y = data[baseI + 8], z = data[baseI + 9]},
                    AngularVelocity = {x = data[baseI + 10], y = data[baseI + 11], z = data[baseI + 12]}
                };
            }
        }

        public override string ToString()
        {
            var result = Root.ToString();

            foreach (var joint in Joints)
            {
                result += $"\t{JointDict[joint]}";
            }

            return result;
        }
    }
}