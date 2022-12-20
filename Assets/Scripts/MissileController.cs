using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MissileController : MonoBehaviour
{
    public GameObject target;
    public float airSpeed = 10;
    public float gain = 3;
    public float maneuverability = 1;

    public Vector3 relativePosition;
    public Vector3 relativeVelocity;
    public Vector3 accelerationCmd;
    public float turnSpeed;
    public float tgo;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        MissileMovement();
    }

    void MissileMovement()
    {
        // get target position and velocity in inertial world frame
        Vector3 targetPosition = target.transform.position;
        Vector3 targetVelocity = target.transform.forward * target.GetComponent<PlayerController>().airSpeed;
        // get missile position and velocity in intertial world frame
        Vector3 missilePosition = transform.position;
        Vector3 missileVelocity = transform.forward * airSpeed;
        // get instantaneous relative position and velocity in world frame (note, relative rotation of local frame and world frame means the frame is relevant!)
        relativePosition = targetPosition - missilePosition;
        relativeVelocity = targetVelocity - missileVelocity;

        tgo = -relativePosition.magnitude * relativePosition.magnitude / Vector3.Dot(relativePosition, relativeVelocity);


        // get acceleration command and body rotation rate (in world frame)
        accelerationCmd = PureProNav(relativePosition, relativeVelocity, gain);// + Vector3.up * 9.8f * gain/2;
        Vector3 bodyRate = Vector3.Cross(transform.forward, accelerationCmd / airSpeed);

        bodyRate = RestrictBodyRate(bodyRate);

        // convert to local frame
        Vector3 bodyRateLocal = transform.InverseTransformVector(bodyRate);

        // Apply missile movement
        transform.Translate(Vector3.forward * airSpeed * Time.deltaTime);
        transform.Rotate(bodyRateLocal * Mathf.Rad2Deg * Time.deltaTime);//
    }

    Vector3 PureProNav(Vector3 relPos, Vector3 relVel, float gain)
    {
        Vector3 Vm = transform.forward * airSpeed;
        Vector3 LoSRate = Vector3.Cross(relPos, relVel) / (relPos.magnitude * relPos.magnitude);

        float speedRatio = relVel.magnitude / Vm.magnitude;

        Vector3 accelerationCmd = -gain * speedRatio * Vector3.Cross(Vm, LoSRate);

        return accelerationCmd;
    }

    Vector3 RestrictBodyRate(Vector3 bodyRate)
    {
        turnSpeed = bodyRate.magnitude;
        float pitchLimit = maneuverability * airSpeed;

        if (turnSpeed > pitchLimit * Mathf.Deg2Rad)
        {
            return bodyRate.normalized * pitchLimit * Mathf.Deg2Rad;
        }
        return bodyRate;
    }
}
