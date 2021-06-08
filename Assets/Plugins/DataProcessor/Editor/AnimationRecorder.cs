using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.Assertions;

public class AnimationRecorder : MonoBehaviour
{
    public DefaultAsset folder;
    private string folderPath;

    public Animator animator;

    public List<AnimationClip> animList = new List<AnimationClip>();

    public AnimatorOverrideController tOverrideController;

    public int timeScale = 1;
    public float fps = 60f;
    private static float dt;
    private float currentTime = 0f;

    public List<ArticulationBody> JointABs;
    public List<Transform> JointTransforms;

    private void Start()
    {
        Time.timeScale = timeScale;
        Application.targetFrameRate = 30 * timeScale;
        dt = 1f / fps;
        JointABs = animator.GetComponentsInChildren<ArticulationBody>().ToList();
        Assert.IsTrue(JointABs[0].isRoot);
        JointTransforms = JointABs.Select(p => p.transform).ToList();
    }

    public void LoadFolder()
    {
        if (tOverrideController == default)
        {
            tOverrideController = new AnimatorOverrideController(animator.runtimeAnimatorController);
            animator.runtimeAnimatorController = tOverrideController;
        }

        folderPath = AssetDatabase.GetAssetPath(folder);
        GetAllFilesInDirectory(folderPath);

        StartCoroutine(CaptureTransform());
    }

    private void GetAllFilesInDirectory(string dirPath)
    {
        var info = new DirectoryInfo(dirPath);
        var fileInfo = info.GetFiles("*.fbx", SearchOption.AllDirectories);

        animList.Clear();

        foreach (var file in fileInfo)
        {
            var absolutePath = file.FullName;
            absolutePath = absolutePath.Replace(Path.DirectorySeparatorChar, '/');
            var relativePath = "";
            if (absolutePath.StartsWith(Application.dataPath))
                relativePath = "Assets" + absolutePath.Substring(Application.dataPath.Length);
            var fbxFile = AssetDatabase.LoadAssetAtPath<GameObject>(relativePath);

            var clips = AssetDatabase.LoadAllAssetRepresentationsAtPath(relativePath)
                .Where(p => p as AnimationClip != null);

            foreach (var clip in clips)
            {
                var animClip = clip as AnimationClip;

                if (animClip != default && animClip.isHumanMotion)
                    animList.Add(animClip);
            }
        }
    }

    private IEnumerator CaptureTransform()
    {
        var t = animator.transform;
        // animator.speed = 0f;


        foreach (var anim in animList)
        {
            print(anim.name);

            t.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            tOverrideController["Handshake_001"] = anim;

            // Set Info
            var motionData = ScriptableObject.CreateInstance<MotionData>();
            motionData.Init(2 + (int) (anim.length / dt));
            motionData.characterName = t.parent.name;
            motionData.motionName = anim.name;
            motionData.fps = fps;

            int frame = 0;

            while (currentTime < anim.length)
            {
                animator.Play("CurrentMotion", 0, currentTime / anim.length);

                yield return new WaitForEndOfFrame();

                var skeletonData = GetSkeletonData(frame++);
                motionData.data.Add(skeletonData);

                currentTime += dt;
            }

            CalculateVelocity(motionData);

            motionData.Save();

            currentTime = 0f;
        }
    }

    private void CalculateVelocity(MotionData motionData)
    {
        #region Root

        var firstPoseJoints = motionData.data[0].joints;
        var secondPose = motionData.data[1].joints;
        firstPoseJoints[0].velocity = GetVelocity(firstPoseJoints[0].position, secondPose[0].position, dt);
        firstPoseJoints[0].angularVelocity =
            GetAngularVelocity(firstPoseJoints[0].rotation, secondPose[0].rotation, dt);

        var middleSize = motionData.data.Count - 1;
        for (var i = 1; i < middleSize; ++i)
        {
            var prevPose = motionData.data[i - 1].joints;
            var curPose = motionData.data[i].joints;
            var nextPose = motionData.data[i + 1].joints;

            curPose[0].velocity = GetVelocity(prevPose[0].position, nextPose[0].position, 2 * dt);
            curPose[0].angularVelocity =
                GetAngularVelocity(prevPose[0].rotation, nextPose[0].rotation, 2 * dt);
        }

        var beforeLastPose = motionData.data[middleSize - 1].joints;
        var lastPose = motionData.data[middleSize].joints;
        lastPose[0].velocity = GetVelocity(beforeLastPose[0].position, lastPose[0].position, dt);
        lastPose[0].angularVelocity =
            GetAngularVelocity(beforeLastPose[0].rotation, lastPose[0].rotation, dt);

        #endregion Root

        for (var index = 1; index < firstPoseJoints.Length; index++)
        {
            firstPoseJoints[index].velocity = GetVelocity(firstPoseJoints[index].position,
                secondPose[index].position, dt);
            firstPoseJoints[index].angularVelocity = GetAngularVelocity(firstPoseJoints[index].rotation,
                secondPose[index].rotation, dt);

            middleSize = motionData.data.Count - 1;
            for (var i = 1; i < middleSize; ++i)
            {
                var prevPose = motionData.data[i - 1].joints;
                var curPose = motionData.data[i].joints;
                var nextPose = motionData.data[i + 1].joints;

                curPose[index].velocity = GetVelocity(prevPose[index].position,
                    nextPose[index].position, 2 * dt);
                curPose[index].angularVelocity = GetAngularVelocity(prevPose[index].rotation,
                    nextPose[index].rotation, 2 * dt);
            }

            lastPose[index].velocity = GetVelocity(beforeLastPose[index].position,
                lastPose[index].position, dt);
            lastPose[index].angularVelocity = GetAngularVelocity(beforeLastPose[index].rotation,
                lastPose[index].rotation, dt);
        }
    }

    private SkeletonData GetSkeletonData(int frameNumber)
    {
        var data = new SkeletonData(JointTransforms.Count) {frameNumber = frameNumber};

        var root = JointTransforms[0];
        var rootPos = root.position;
        var rootRot = root.rotation;
        var rootInv = Quaternion.Inverse(rootRot);

        data.joints[0] = new JointData {position = rootPos, rotation = rootRot, jointIdx = 0, jointName = root.name};

        // TODO: Fix center of mass calculation
        data.centerOfMass = (JointTransforms[0].position - rootPos) * JointABs[0].mass;
        var totalMass = JointABs[0].mass;
        Debug.DrawLine(Vector3.zero, root.position, Color.red, 0.1f);


        for (var index = 1; index < data.joints.Length; index++)
        {
            var jointTransform = JointTransforms[index];

            var relativePos = rootInv * (jointTransform.position - rootPos);
            var relativeRot = rootInv * jointTransform.rotation;

            data.joints[index] = new JointData
                {position = relativePos, rotation = relativeRot, jointIdx = index, jointName = jointTransform.name};
            data.centerOfMass += (JointTransforms[index].position - rootPos) * JointABs[index].mass;
            totalMass += JointABs[index].mass;
        }

        data.centerOfMass /= totalMass;

        return data;
    }

    private static Vector3 GetVelocity(Vector3 from, Vector3 to, float deltaTime)
    {
        return (to - from) / deltaTime;
    }

    private static Vector3 GetAngularVelocity(Quaternion from, Quaternion to, float deltaTime)
    {
        var q = to * Quaternion.Inverse(from);

        if (Mathf.Abs(q.w) > 0.999999f)
            return new Vector3(0, 0, 0);

        float gain;
        if (q.w < 0.0f)
        {
            var angle = Mathf.Acos(-q.w);
            gain = -2.0f * angle / (Mathf.Sin(angle) * deltaTime);
        }
        else
        {
            var angle = Mathf.Acos(q.w);
            gain = 2.0f * angle / (Mathf.Sin(angle) * deltaTime);
        }

        return new Vector3(q.x * gain, q.y * gain, q.z * gain);
    }


    private void OnGUI()
    {
        GUILayout.Label($"{1 / Time.deltaTime} fps");
    }
}