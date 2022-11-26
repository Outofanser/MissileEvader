using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerController : MonoBehaviour
{
    public float pitchRate = 20; // deg/s
    public float rollRate = 30;
    public float yawRate = 10;
    public float airSpeed = 30;
    public GameObject centerOfMassObject;

    // Make these private
    public Quaternion currentRotation;
    public Quaternion attitudeRate;
    public Vector3 acceleration;
    private Vector3 centerOfMass;

    // Start is called before the first frame update
    void Start()
    {
        currentRotation = transform.rotation;
        centerOfMass = centerOfMassObject.transform.localPosition;
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 playerAttitudeInput = new Vector3(-Input.GetAxis("Pitch"), -Input.GetAxis("Yaw"), -Input.GetAxis("Roll"));

        attitudeRate.eulerAngles = playerAttitudeInput;
        acceleration = Vector3.Cross(Vector3.forward,attitudeRate.eulerAngles) * airSpeed; // may be useful information to use truth data with missile system

        currentRotation.eulerAngles = Vector3.Scale(playerAttitudeInput, new Vector3(pitchRate,yawRate,rollRate)*Time.deltaTime);

        transform.rotation = currentRotation;
        transform.position += Vector3.forward * airSpeed;



        

    }
}
