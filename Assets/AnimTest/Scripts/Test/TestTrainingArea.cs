using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestTrainingArea : MonoBehaviour
{
    public GameObject testAgentPrefab;
    // Start is called before the first frame update
    void Start()
    {
        
#if UNITY_EDITOR
        int size = 1;
#else
        int size = 5;
#endif
        int gap = 20;
        for (var i = 0; i < size; ++i)
        for (var j = 0; j < size; ++j)
            Instantiate(testAgentPrefab, new Vector3(
                gap / 2 * (1 - size + 2 * i),
                0,
                gap * -j
            ), Quaternion.identity, transform);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
