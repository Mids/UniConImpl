using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace DataProcessor
{
    public class MotionData
    {
        public List<SkeletonData> Data;

        public MotionData(List<SkeletonData> data)
        {
            Data = data;
        }

        public MotionData(int totalFrame)
        {
            Data = new List<SkeletonData>(totalFrame);
        }

        public SkeletonData GetPose(int frame)
        {
            if (frame < Data.Count) return Data[frame];

            Debug.LogWarning("Calling old frame");
            return Data[Data.Count - 1];
        }

        public override string ToString()
        {
            return Data.Aggregate("", (current, datum) => current + $"{datum}\n");
        }
    }
}