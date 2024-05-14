using System.Collections;
using System.Collections.Generic;
using UnityEditor.ShaderGraph;
using UnityEngine;

public class MissileController : MonoBehaviour
{
    public GameObject target;
    public Rigidbody body;
    public float airSpeed = 50;
    public float gain = 3;
    public float aoa_limit_deg = 10;
    public float maneuverability = 10;
    public float control_limit_degps = 1;
    

    public Vector3 relativePosition;
    public Vector3 relativeVelocity;
    public Vector3 accelerationCmd;
    public float turnSpeed;
    public float tgo;
    public float attack_limited;
    public float attackAngle;
    public Vector3 lift_direction;
    public Vector3 local_lift;
    public Vector3 control;

    // Start is called before the first frame update
    void Start()
    {
        body = GetComponent<Rigidbody>();
        body.AddForce(Vector3.up * airSpeed, ForceMode.VelocityChange);
    }

    // Update is called once per frame
    void Update()
    {
        //MissileMovement();
    }

    void FixedUpdate()
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
        ///Vector3 missileVelocity = transform.forward * airSpeed;
        Vector3 missileVelocity = body.velocity;
        // get instantaneous relative position and velocity in world frame (note, relative rotation of local frame and world frame means the frame is relevant!)
        relativePosition = targetPosition - missilePosition;
        relativeVelocity = targetVelocity - missileVelocity;

        tgo = -relativePosition.magnitude * relativePosition.magnitude / Vector3.Dot(relativePosition, relativeVelocity);


        // get acceleration command and body rotation rate (in world frame)
  
        accelerationCmd = PureProNav(missileVelocity,relativePosition, relativeVelocity, gain);// + Vector3.up * 9.8f * gain/2;

        accelerationCmd += -Physics.gravity;

        //Vector3 bodyRate = Vector3.Cross(transform.forward, accelerationCmd / airSpeed);
        //bodyRate = RestrictBodyRate(bodyRate);
        //
        // convert to local frame
        //Vector3 bodyRateLocal = transform.InverseTransformVector(bodyRate);
        //
        // Apply missile movement
        //transform.Translate(Vector3.forward * airSpeed * Time.deltaTime);
        //transform.Rotate(bodyRateLocal * Mathf.Rad2Deg * Time.deltaTime);//
        //

        Vector3 forward = body.transform.forward;

        Vector3 temp1 = Vector3.Cross(forward, missileVelocity);
        Vector3 temp2 = Vector3.Cross(missileVelocity, temp1);
        lift_direction = temp2.normalized;

        float vel = missileVelocity.magnitude;
        float area = 0.01f;
        attackAngle = Vector3.Angle(forward, missileVelocity);

        float dAoA = accelerationCmd.magnitude / area / vel / vel / (2 * Mathf.PI);
        attack_limited = Mathf.Min(dAoA*Mathf.Rad2Deg, aoa_limit_deg);
        Vector3 turnDir = Vector3.Cross(missileVelocity, accelerationCmd).normalized;


        float lift_coefficient = 2 * Mathf.PI * attackAngle * Mathf.Deg2Rad;
        float drag_coefficient = 2 * Mathf.PI * Mathf.Pow(attackAngle*Mathf.Deg2Rad,4f);
        if (attackAngle > 12)
        {
            lift_coefficient = Mathf.Sin(2f*attackAngle*Mathf.Deg2Rad);
            drag_coefficient = 1f - Mathf.Cos(2f*attackAngle*Mathf.Deg2Rad);
        }


        Vector3 lift_force = lift_direction * lift_coefficient * vel * vel * area;
        Vector3 drag_force = -missileVelocity.normalized * drag_coefficient * vel * vel * area;
 
        Vector3 desiredForward = Quaternion.AngleAxis(attack_limited, turnDir) * missileVelocity.normalized;
        Vector3 angle = -Vector3.Cross(desiredForward, forward);
        float phi = Mathf.Asin(angle.magnitude)/Time.fixedDeltaTime;
        control = (phi * angle.normalized)/Time.fixedDeltaTime - body.angularVelocity;
        control = control * Mathf.Min(control_limit_degps, control.magnitude) / control.magnitude;
        

        //body.AddRelativeForce(local_lift,ForceMode.Acceleration);// + drag_force);
        body.AddForce(lift_force+drag_force,ForceMode.Acceleration);

        //body.AddForce(lift_force, ForceMode.Acceleration);

        if (control.magnitude > 0.001)
        {
            body.AddTorque(control, ForceMode.Acceleration);
        }
    }

    Vector3 PureProNav(Vector3 Vm, Vector3 relPos, Vector3 relVel, float gain)
    {
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
