using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;

[RequireComponent(typeof(BehaviorParameters))]
public class TestAgent : Agent
{
    private List<ArticulationBody> AgentABs;

    public int power = 100;

    private float highest;

    public override void Initialize()
    {
        base.Initialize();

        AgentABs = GetComponentsInChildren<ArticulationBody>().ToList();
    }

    public override void OnEpisodeBegin()
    {
        AgentABs[0].TeleportRoot(transform.position + new Vector3(0f, 4.5f, 0f), Quaternion.identity);
        AgentABs[1].jointPosition = new ArticulationReducedSpace(0, 0);
        AgentABs[2].jointPosition = new ArticulationReducedSpace(0, 0);

        highest = 4.5f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(AgentABs[0].transform.localPosition);
        sensor.AddObservation(AgentABs[0].transform.localRotation);
        sensor.AddObservation(AgentABs[1].transform.localPosition);
        sensor.AddObservation(AgentABs[1].transform.localRotation);
        sensor.AddObservation(AgentABs[2].transform.localPosition);
        sensor.AddObservation(AgentABs[2].transform.localRotation);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var arr = actions.ContinuousActions.Array;

        for (int i = 0; i < arr.Length; i++) arr[i] *= power;

        AgentABs[1].AddRelativeTorque(new Vector3(0, arr[0], arr[1]));
        AgentABs[2].AddRelativeTorque(new Vector3(0, arr[2], arr[3]));


        var curHigh = AgentABs[0].transform.position.y;
        if (curHigh > highest)
        {
            AddReward(curHigh * curHigh - highest * highest);
            highest = curHigh;
        }

        if (AgentABs[0].transform.position.y < 0.75f || AgentABs[1].transform.position.y < 0.75f)
        {
            AddReward(-1f);
            EndEpisode();
            return;
        }

        AddReward(0.01f);
    }


    public override void Heuristic(in ActionBuffers actionsOut)
    {
        for (var index = 0; index < actionsOut.ContinuousActions.Array.Length; index++)
            actionsOut.ContinuousActions.Array[index] = Random.Range(-1f, 1f);
    }

    public void FixedUpdate()
    {
        RequestDecision();
    }
}