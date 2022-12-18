using Microsoft.Win32.SafeHandles;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerController : MonoBehaviour
{
    public float pitchRate = 20; // deg/s
    public float rollRate = 30;
    public float yawRate = 10;
    public float airSpeed = 30; // m/s

    // Make these private
    public Vector3 acceleration;


    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        PlayerMovement();
        ApplyBoundary();

    }

    void PlayerMovement()
    {
        // Move player forward in forward direction (not realistic physics)
        transform.Translate(Vector3.forward * airSpeed * Time.deltaTime);

        // get inputs to attitude controls
        Vector3 playerAttitudeInput = new Vector3(-Input.GetAxis("Pitch"), -Input.GetAxis("Yaw"), -Input.GetAxis("Roll"));
        // scale inputs to body moments (body rates)
        Vector3 rotationRate = Vector3.Scale(playerAttitudeInput, new Vector3(pitchRate, yawRate, rollRate));

        acceleration = Vector3.Cross(Vector3.forward * airSpeed, rotationRate); // may be useful information to use truth data with missile system

        // Rotate player via inputs
        transform.Rotate(rotationRate * Time.deltaTime);
    }

    void ApplyBoundary()
    {
        transform.position = new Vector3(MinMax(-500f,500f,transform.position.x), MinMax(0f, 200f, transform.position.y),MinMax(-500f,500f,transform.position.z));
    }

    float MinMax(float minVal, float maxVal, float compVal)
    {
        return Mathf.Max(minVal, Mathf.Min(compVal, maxVal));
    }

}
