using System.Collections.Generic;
using System.IO;
using System.IO.Abstractions.TestingHelpers;
using UnityEngine;

namespace DataProcessor
{
    public class AnimationImporter : MonoBehaviour
    {
        public static MotionData Import(string path)
        {
            var sr = new StreamReader(path);
            var skeletonData = new List<SkeletonData>(100);

            var jointLine = sr.ReadLine();
            while (true)
            {
                var line = sr.ReadLine();
                if (line == default)
                    break;

                var sd = new SkeletonData();
                sd.InitDict(jointLine);
                sd.InitData(line);
                skeletonData.Add(sd);
            }

            sr.Close();

            return new MotionData(skeletonData);
        }

        public static MotionData Import(TextAsset textAsset)
        {
            var lines = textAsset.text.SplitLines();
            var skeletonData = new List<SkeletonData>(100);

            var jointLine = lines[0];
            for (var i = 1; i < lines.Length; ++i)
            {
                var sd = new SkeletonData();
                sd.InitDict(jointLine);
                sd.InitData(lines[i]);
                skeletonData.Add(sd);
            }

            return new MotionData(skeletonData);
        }
    }
}