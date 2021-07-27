using UnityEngine;

public class FootContact : MonoBehaviour
{
    public bool isContact = false;

    private void OnCollisionEnter(Collision other)
    {
        if (other.gameObject.name == "Floor") isContact = true;
    }

    private void OnCollisionExit(Collision other)
    {
        if (other.gameObject.name == "Floor") isContact = false;
    }
}