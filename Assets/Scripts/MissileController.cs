using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using Unity.VisualScripting;
using Unity.VisualScripting.FullSerializer;
using UnityEngine;

public class MissileController : MonoBehaviour
{
    public GameObject target;
    private Vector3 targetPos;
    public Rigidbody body;
    [SerializeField] GameObject missileCOM;
    public float airSpeed = 50;
    public float gain = 3;
    public float aoa_limit_deg = 10;
    public float maneuverability = 10;
    public float control_limit_degps = 5;

    private float rho = 1.293f;
    private float area = 1f;
    //private float bodyLength = 1f;

    public bool armed;
    public bool exploded;
    public float tgo;
    public float attack_limited;
    public float attackAngle;
    public bool gravity_turn = true;

    public float pGain = 1f;
    public float dGain = 0.1f;
    public float iGain = 1f;

    public ParticleSystem explosionParticle;
    public AudioClip explosionSound;
    private AudioSource missileAudio;
    private IEnumerator thrustLooper;
    public AudioClip thrustSound;

    private ParticleSystem missilePs;

    private Vector3 errorLast = Vector3.zero;
    private Vector3 errorAccumulator = Vector3.zero;

    public float vel;
    private Vector3 relativePosition;
    public float distance;

    // Start is called before the first frame update
    void Start()
    {
        body = GetComponent<Rigidbody>();
        body.centerOfMass = missileCOM.transform.localPosition;
        body.AddForce(Vector3.up * airSpeed, ForceMode.VelocityChange);
        missileAudio = GetComponent<AudioSource>();
        thrustLooper = LoopAudio(1f);
        missileAudio.clip = thrustSound;


        //missileAudio.volume = 0.1f;
        //missileAudio.loop = true;

        StartCoroutine(thrustLooper);


        explosionParticle.Stop();
        //StartCoroutine(thrustLooper);
    }

    // Update is called once per frame
    void Update()
    {
        //MissileMovement()
        targetPos = target.GetComponent<PlayerController>().centerOfMass.position;
    }

    void FixedUpdate()
    {
        MissileMovement();


    }

    void MissileMovement()
    {
        // get target position and velocity in inertial world frame
        Vector3 targetPosition = targetPos;
        Vector3 targetVelocity = target.transform.forward * target.GetComponent<PlayerController>().airSpeed;
        // get missile position and velocity in intertial world frame
        Vector3 missilePosition = transform.position;
        ///Vector3 missileVelocity = transform.forward * airSpeed;
        Vector3 missileVelocity = body.linearVelocity;
        // get instantaneous relative position and velocity in world frame (note, relative rotation of local frame and world frame means the frame is relevant!)
        relativePosition = targetPosition - missilePosition;
        Vector3 relativeVelocity = targetVelocity - missileVelocity;
        distance = relativePosition.magnitude;

        tgo = -relativePosition.magnitude * relativePosition.magnitude / Vector3.Dot(relativePosition, relativeVelocity);

        if (tgo < 10 && !gravity_turn)
        {
            armed = true;
        }
        if (tgo < 0 && armed)
        {
            Explode();
        }

        // get acceleration command and body rotation rate (in world frame)
        Vector3 accelerationCmd = PureProNav(missileVelocity, relativePosition, relativeVelocity, gain);// + Vector3.up * 9.8f * gain/2;
        accelerationCmd += -Physics.gravity;
        if (gravity_turn)
        {
            Vector3 rotationDir = Vector3.Cross(missileVelocity, relativePosition).normalized;
            accelerationCmd = Vector3.Cross(rotationDir, missileVelocity).normalized * 9.8f;
            if (accelerationCmd.normalized.y > 0 || accelerationCmd.normalized.y < -0.99f)
            {
                gravity_turn = false;
            }
        }

        Vector3 forward = body.transform.forward;

        vel = missileVelocity.magnitude;
        float dynamicPressure = 0.5f * rho * vel * vel;

        float dAoA = accelerationCmd.magnitude * body.mass / area / dynamicPressure / (2 * Mathf.PI);
        attack_limited = Mathf.Min(dAoA * Mathf.Rad2Deg, aoa_limit_deg);
        Vector3 turnDir = Vector3.Cross(missileVelocity, accelerationCmd).normalized;


        Vector3 desiredForward = Quaternion.AngleAxis(attack_limited, turnDir) * missileVelocity.normalized;
        Vector3 angle = -Vector3.Cross(desiredForward, forward);
        float phi = Mathf.Asin(angle.magnitude) / Time.fixedDeltaTime;
        ///Vector3 control = 1/100f*(phi * angle.normalized - body.angularVelocity)/Time.fixedDeltaTime*body.mass;

        Vector3 control = PIDController(Vector3.zero, phi * angle.normalized);

        Vector3 torque = GenerateControlTorque(control, missileVelocity, 0.1f);


        //body.AddRelativeForce(local_lift,ForceMode.Acceleration);// + drag_force);
        Vector3 aeroForces = GenerateAeroForces(forward, missileVelocity, area);
        body.AddForce(aeroForces);

        //body.AddForce(lift_force, ForceMode.Acceleration);

        if (torque.magnitude > 0.001)
        {
            body.AddTorque(torque);
        }
    }

    Vector3 PureProNav(Vector3 Vm, Vector3 relPos, Vector3 relVel, float gain)
    {
        Vector3 LoSRate = Vector3.Cross(relPos, relVel) / (relPos.magnitude * relPos.magnitude);

        float speedRatio = relVel.magnitude / Vm.magnitude;

        Vector3 accelerationCmd = -gain * speedRatio * Vector3.Cross(Vm, LoSRate);

        return accelerationCmd;
    }

    Vector3 GenerateAeroForces(Vector3 forward, Vector3 missileVelocity, float wingArea)
    {

        Vector3 temp1 = Vector3.Cross(forward, missileVelocity);
        Vector3 temp2 = Vector3.Cross(missileVelocity, temp1);
        Vector3 lift_direction = temp2.normalized;

        float vel = missileVelocity.magnitude;
        float dynamicPressure = 0.5f * rho * vel * vel;
        attackAngle = Vector3.Angle(forward, missileVelocity);

        float lift_coefficient = 2f * Mathf.PI * attackAngle * Mathf.Deg2Rad;
        float drag_coefficient = 1f * Mathf.Pow(attackAngle * Mathf.Deg2Rad, 2f);
        //float moment_coefficient = 0.01f * attackAngle * Mathf.Deg2Rad;
        if (attackAngle > 12)
        {
            lift_coefficient = Mathf.Sin(2f * attackAngle * Mathf.Deg2Rad);
            drag_coefficient = 1f - Mathf.Cos(2f * attackAngle * Mathf.Deg2Rad);
        }


        Vector3 lift_force = lift_direction * lift_coefficient * dynamicPressure * area;
        Vector3 drag_force = -missileVelocity.normalized * drag_coefficient * dynamicPressure * area;

        return lift_force + drag_force;
    }

    Vector3 GenerateControlTorque(Vector3 control, Vector3 missileVelocity, float tailArea)
    {
        float max_deflect = 10f;
        float tail2CMDistance = 0.5f;

        float vel = missileVelocity.magnitude;
        float dynamicPressure = 0.5f * rho * vel * vel;
        float lift_coefficient = 2f * Mathf.PI * max_deflect;

        float max_torque = dynamicPressure * lift_coefficient * tailArea * tail2CMDistance;
        Vector3 desired_torque = control * max_torque;

        return desired_torque;

    }

    Vector3 PIDController(Vector3 startOrientation, Vector3 endOrientation)
    {
        Vector3 error = (endOrientation - startOrientation) / 2f / Mathf.PI;
        Vector3 dError = (error - errorLast) / Time.fixedDeltaTime;
        Vector3 iError = errorAccumulator + error * Time.fixedDeltaTime;

        if (errorLast == Vector3.zero)
        {
            dError = Vector3.zero;
        }

        Vector3 PID = pGain * error + dGain * dError + iGain * iError;

        if (PID.magnitude > 1)
        {
            PID = PID.normalized;
        }

        errorLast = error;
        errorAccumulator += error;


        return PID;
    }

    private void Explode()
    {
        if (!exploded)
        {
            Damage();
            exploded = true;
            StopCoroutine(thrustLooper);
            explosionParticle.Play();
            missileAudio.Stop();
            missileAudio.PlayOneShot(explosionSound, 0.2f);
            Destroy(gameObject.transform.GetChild(0).gameObject);
            StartCoroutine(Explosion());
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Player"))
        {
            Explode();
        }
        else if (collision.gameObject.CompareTag("Ground"))
        {
            Explode();
        }
    }

    IEnumerator Explosion()
    {
        yield return new WaitForSeconds(2f);
        Destroy(gameObject);
    }

    private void Damage()
    {
        float distance = (targetPos - transform.position).magnitude;

        float damage = 300f / distance;

        if (damage < 10f)
        {
            return;
        }

        target.GetComponent<PlayerController>().health -= Mathf.Min(damage, 80f);
        Debug.Log("Player hit! Health is now " + target.GetComponent<PlayerController>().health);
    }

    IEnumerator LoopAudio(float waitTime)
    {
        while (true)
        {
            float mywait = Mathf.Max(0.1f, Mathf.Min(1f, distance / 1000f)) * waitTime;
            missileAudio.time = 0f;
            missileAudio.Play();
            yield return new WaitForSeconds(mywait);
        }

    }


}
