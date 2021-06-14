using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;

public class AnimationPlayer : MonoBehaviour
{
    public MotionData motion;

    public List<ArticulationBody> JointABs;
    public List<Transform> JointTransforms;

    public float dt;

    public void PlayCoroutine()
    {
        dt = 1f / motion.fps;
        JointABs = GetComponentsInChildren<ArticulationBody>().ToList();
        if (JointABs.Count > 0)
        {
            Assert.IsTrue(JointABs[0].isRoot);
            JointTransforms = JointABs.Select(p => p.transform).ToList();
            foreach (var jointAB in JointABs) jointAB.enabled = false;
        }

        StartCoroutine(Play());
    }

    private IEnumerator Play()
    {
        foreach (var skeleton in motion.data)
        {
            var rootPos = transform.position + skeleton.joints[0].position;
            var rootRot = skeleton.joints[0].rotation;

            JointTransforms[0].position = rootPos;
            JointTransforms[0].rotation = rootRot;

            for (var index = 1; index < JointTransforms.Count; index++)
            {
                JointTransforms[index].position = rootPos + rootRot * skeleton.joints[index].position;
                JointTransforms[index].rotation = rootRot * skeleton.joints[index].rotation;
            }

            yield return new WaitForSeconds(dt);
        }
    }
}