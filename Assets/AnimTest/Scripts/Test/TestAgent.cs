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

    public int power = 1000;

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
        AgentABs[1].jointVelocity = new ArticulationReducedSpace(0, 0);
        AgentABs[2].jointPosition = new ArticulationReducedSpace(0, 0);
        AgentABs[2].jointVelocity = new ArticulationReducedSpace(0, 0);


        foreach (var agentAB in AgentABs)
        {
            agentAB.velocity = Vector3.zero;
            agentAB.angularVelocity = Vector3.zero;
        }

        highest = 4.5f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        foreach (var agentAB in AgentABs)
        {
            sensor.AddObservation(agentAB.transform.localPosition);
            sensor.AddObservation(agentAB.transform.localRotation);
            sensor.AddObservation(agentAB.velocity);
            sensor.AddObservation(agentAB.angularVelocity);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var arr = actions.ContinuousActions.Array;

        for (int i = 0; i < arr.Length; i++) arr[i] *= power;

        var localTorque1 = AgentABs[1].parentAnchorRotation * new Vector3(0f, arr[0], arr[1]);
        AgentABs[1].AddRelativeTorque(localTorque1);

        var localTorque2 = AgentABs[2].parentAnchorRotation * new Vector3(0f, arr[2], arr[3]);
        AgentABs[2].AddRelativeTorque(localTorque2);


        var curHigh = AgentABs[0].transform.position.y;
        if (curHigh > highest)
        {
            AddReward(curHigh - highest);
            highest = curHigh;
        }

        if (AgentABs[0].transform.position.y < 0.85f || AgentABs[1].transform.position.y < 0.85f)
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