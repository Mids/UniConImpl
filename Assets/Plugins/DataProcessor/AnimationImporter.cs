using System.Collections.Generic;
using System.IO;
using System.IO.Abstractions.TestingHelpers;
using System.Linq;
using UnityEngine;

namespace DataProcessor
{
    public class AnimationImporter : MonoBehaviour
    {
        public static MotionData Import(string path)
        {
            var sr = new StreamReader(path);
            var skeletonData = new List<SkeletonData>(100);

            while (true)
            {
                var line = sr.ReadLine();
                if (line == default)
                    break;

                skeletonData.Add(new SkeletonData(line));
            }

            sr.Close();

            return new MotionData(skeletonData);
        }

        public static MotionData Import(TextAsset textAsset)
        {
            var lines = textAsset.text.SplitLines();
            var skeletonData = new List<SkeletonData>(100);

            skeletonData.AddRange(
                lines.TakeWhile(line => line != default).Select(line => new SkeletonData(line)));

            return new MotionData(skeletonData);
        }
    }
}