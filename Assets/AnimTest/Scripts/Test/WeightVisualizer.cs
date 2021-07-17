using Unity.MLAgents.Policies;
using UnityEngine;

[RequireComponent(typeof(BehaviorParameters))]
public class WeightVisualizer : MonoBehaviour
{
#if UNITY_EDITOR
    private BehaviorParameters _behaviorParameters;

    // Start is called before the first frame update
    private void Start()
    {
        _behaviorParameters = GetComponent<BehaviorParameters>();
        var v = _behaviorParameters.Model.modelData;

        
    }

    // Update is called once per frame
    private void Update()
    {
    }
#endif
}