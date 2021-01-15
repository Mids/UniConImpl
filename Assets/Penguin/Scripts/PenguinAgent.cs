using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class PenguinAgent : Agent
{
    [Tooltip("How fast the agent moves forward")]
    public float moveSpeed = 5f;

    [Tooltip("How fast the agent turns")]
    public float turnSpeed = 180f;

    [Tooltip("Prefab of the heart that appears when the baby is fed")]
    public GameObject heartPrefab;

    [Tooltip("Prefab of the regurgitated fish that appears when the baby is fed")]
    public GameObject regurgitatedFishPrefab;
    
    
    private PenguinArea penguinArea;
    private new Rigidbody rigidbody;
    private GameObject baby;
    private bool isFull; // If true, penguin has a full stomach

    private Vector3 _prev = Vector3.zero;
    
    /// <summary>
    /// Initial setup, called when the agent is enabled
    /// </summary>
    public override void Initialize()
    {
        base.Initialize();
        penguinArea = GetComponentInParent<PenguinArea>();
        baby = penguinArea.penguinBaby;
        rigidbody = GetComponent<Rigidbody>();
    }
    
    /// <summary>
    /// Perform actions based on a vector of numbers
    /// </summary>
    /// <param name="actionBuffers">The struct of actions to take</param>
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Convert the first action to forward movement
        float forwardAmount = 1;//actionBuffers.ContinuousActions[0];
        // forwardAmount = forwardAmount > 0 ? forwardAmount : 0;

        // Convert the second action to turning left or right
        float turnAmount = actionBuffers.ContinuousActions[0];

        // Apply movement
        rigidbody.MovePosition(transform.position + transform.forward * forwardAmount * moveSpeed * Time.fixedDeltaTime);
        transform.Rotate(transform.up * turnAmount * turnSpeed * Time.fixedDeltaTime);

        // Apply a tiny negative reward every step to encourage action
        if (MaxStep > 0) AddReward(-1f / MaxStep);

        var position = transform.position;
        if(Vector3.Distance(_prev, position) < 0.08)
            AddReward(-0.005f);
        _prev = position;

    }
    
    
    /// <summary>
    /// Read inputs from the keyboard and convert them to a list of actions.
    /// This is called only when the player wants to control the agent and has set
    /// Behavior Type to "Heuristic Only" in the Behavior Parameters inspector.
    /// </summary>
    /// <returns>A vectorAction array of floats that will be passed into <see cref="AgentAction(float[])"/></returns>
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        float forwardAction = 0;
        float turnAction = 0;
        if (Input.GetKey(KeyCode.W))
        {
            // move forward
            forwardAction = 1;
        }
        if (Input.GetKey(KeyCode.A))
        {
            // turn left
            turnAction = -1;
        }
        else if (Input.GetKey(KeyCode.D))
        {
            // turn right
            turnAction = 1;
        }

        // Put the actions into the array
        actionsOut.ContinuousActions.Array[0] = forwardAction;
        actionsOut.ContinuousActions.Array[1] = turnAction;
    }
    
    /// <summary>
    /// When a new episode begins, reset the agent and area
    /// </summary>
    public override void OnEpisodeBegin()
    {
        isFull = false;
        penguinArea.ResetArea();
    }
    
    
    /// <summary>
    /// Collect all non-Raycast observations
    /// </summary>
    /// <param name="sensor">The vector sensor to add observations to</param>
    public override void CollectObservations(VectorSensor sensor)
    {
        
        // Whether the penguin has eaten a fish (1 float = 1 value)
        sensor.AddObservation(isFull);

        // // Distance to the baby (1 float = 1 value)
        // sensor.AddObservation(Vector3.Distance(baby.transform.position, transform.position));

        var dir = baby.transform.position - transform.position;
        sensor.AddObservation(dir.magnitude);
        dir.Normalize();
        // Direction to baby (1 Vector3 = 3 values)
        sensor.AddObservation(dir.x);
        sensor.AddObservation(dir.z);

        // Direction penguin is facing (1 Vector3 = 3 values)
        var forward = transform.forward;
        sensor.AddObservation(forward.x);
        sensor.AddObservation(forward.z);
        
        sensor.AddObservation(GetClosestFishDistance());

        // 1 + 1 + 3 + 3 = 6 total values
    }

    private float GetClosestFishDistance()
    {
        float closest = 15f;
        foreach (var o in penguinArea.FishList)
        {
            var temp = Vector3.Distance(o.transform.position, transform.position);
            if (closest > temp) closest = temp;
        }

        return closest;
    }
    
    /// <summary>
    /// When the agent collides with something, take action
    /// </summary>
    /// <param name="collision">The collision info</param>
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.transform.CompareTag("fish"))
        {
            // Try to eat the fish
            EatFish(collision.gameObject);
        }
        else if (collision.transform.CompareTag("baby"))
        {
            // Try to feed the baby
            RegurgitateFish();
        }
    }
    
    /// <summary>
    /// Check if agent is full, if not, eat the fish and get a reward
    /// </summary>
    /// <param name="fishObject">The fish to eat</param>
    private void EatFish(GameObject fishObject)
    {
        if (isFull) {
            AddReward(-0.1f);
            return; // Can't eat another fish while full
        }
        isFull = true;

        penguinArea.RemoveSpecificFish(fishObject);

        AddReward(1f);
    }
    
    
    /// <summary>
    /// Check if agent is full, if yes, feed the baby
    /// </summary>
    private void RegurgitateFish()
    {
        if (!isFull) return; // Nothing to regurgitate
        isFull = false;

        // Spawn regurgitated fish
        GameObject regurgitatedFish = Instantiate<GameObject>(regurgitatedFishPrefab);
        regurgitatedFish.transform.parent = transform.parent;
        regurgitatedFish.transform.position = baby.transform.position;
        Destroy(regurgitatedFish, 4f);

        // Spawn heart
        GameObject heart = Instantiate<GameObject>(heartPrefab);
        heart.transform.parent = transform.parent;
        heart.transform.position = baby.transform.position + Vector3.up;
        Destroy(heart, 4f);

        AddReward(1f);

        if (penguinArea.FishRemaining <= 0)
        {
            EndEpisode();
        }
    }
}