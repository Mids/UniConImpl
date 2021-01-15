using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class TrainingArea : MonoBehaviour
{
    public MannequinAgent mannequinAgent;
    public GameObject mannequinRef;
    private Animator _animatorRef;
    public float AnimationLength;

    public TextMeshPro cumulativeRewardText;
    private static readonly int Reset = Animator.StringToHash("Reset");

    public void ResetArea()
    {
        ResetReference();
        ResetAgent();
    }

    private void ResetReference()
    {
        // var randomValue = Random.Range(0, _animationLength);
        var randomValue = 0;
        _animatorRef.Play("HipHopDancing", 0, randomValue);
    }

    private void ResetAgent()
    {
        
    }
    
    // Start is called before the first frame update
    void Start()
    {
        _animatorRef = mannequinRef.GetComponentInChildren<Animator>();
        AnimationLength = _animatorRef.GetCurrentAnimatorClipInfo(0)[0].clip.length;
        ResetArea();
    }

    // Update is called once per frame
    void Update()
    {
        cumulativeRewardText.text = mannequinAgent.GetCumulativeReward().ToString("0.00");
    }
}
