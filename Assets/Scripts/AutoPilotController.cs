using Unity.VisualScripting;
using UnityEngine;

public class AutoPilotController : MonoBehaviour
{
    //fields
    private GameObject m_target; 
    [SerializeField]
    private float m_timeConstant = 0.8f;
    [SerializeField]
    private float m_angleOfAttackLimit = 10f; // degrees
    private PIDController m_pidController;
    private AerodynamicsModel m_AeroModel;
    private Vector3 m_velocity;
    private Vector3 m_accelerationCmd;
    private Rigidbody m_body;
    public bool IsInPursuit { get; private set; } = false;

    //properties
    public Vector3 RelativePosition { get; private set; }
    public Vector3 RelativeVelocity { get; private set; }
    public float Gain { get; set; } = 3f;
    [field: SerializeField]
    public float Tgo { get; private set; }
    public GameObject Target
    {
        get
        {
            return m_target;
        }
        set
        {
            m_target = value;
        }
    }

    void Awake()
    {
        m_pidController = GetComponent<PIDController>();
        m_AeroModel = GetComponent<AerodynamicsModel>();
        m_body = GetComponent<Rigidbody>();
    }

    void Update()
    {
        Vector3 targetPosition = m_target.GetComponent<PlayerController>().centerOfMass.position;
        Vector3 targetVelocity = m_target.transform.forward * m_target.GetComponent<PlayerController>().airSpeed;
        RelativePosition = targetPosition - transform.position;
        RelativeVelocity = targetVelocity - m_body.linearVelocity;

        m_velocity = m_body.linearVelocity;

        Tgo = -RelativePosition.magnitude * RelativePosition.magnitude / Vector3.Dot(RelativePosition, RelativeVelocity);

        // determine acceleration command Strategy
        if (IsInPursuit)
        {
            m_accelerationCmd = PursuitCommand();
        }
        else
        {
            m_accelerationCmd = GravityTurnCommand();
        }

        // Leave gravity turn when we are stable
        if (!IsInPursuit && (m_accelerationCmd.normalized.y > 0 || m_accelerationCmd.normalized.y < -0.99f))
        {
            IsInPursuit = true;
        }

    }

    public Vector3 GetControl()
    {

        float area = m_AeroModel.WingArea;
        float dynamicPressure = m_AeroModel.DynPressure;

        // Lift Force ~= 2pi * AoA * dynP * area
        float desiredAngleOfAttack = m_accelerationCmd.magnitude * m_body.mass / area / dynamicPressure / (2 * Mathf.PI);
        float attackLimited = Mathf.Min(desiredAngleOfAttack * Mathf.Rad2Deg, m_angleOfAttackLimit);

        // what even is this bit?
        Vector3 turnDir = Vector3.Cross(m_velocity, m_accelerationCmd).normalized;
        Vector3 desiredForward = Quaternion.AngleAxis(attackLimited, turnDir) * m_velocity.normalized;
        Vector3 angle = -Vector3.Cross(desiredForward, m_body.transform.forward);
        float phi = Mathf.Asin(angle.magnitude);

        // get the error rate to calculate the control for this time step
        Vector3 error = phi * angle.normalized / 2f / Mathf.PI;
        Vector3 errorRate = error / m_timeConstant; // Slew control to moderate the error rate
        Vector3 control = m_pidController.PID(errorRate);

        return control;
    }

    Vector3 GravityTurnCommand()
    {

        Vector3 rotationDir = Vector3.Cross(m_velocity, RelativePosition).normalized;
        Vector3 accelerationCmd = Vector3.Cross(rotationDir, m_velocity).normalized * 9.8f;
        return accelerationCmd;
        
    }

    Vector3 PursuitCommand()
    {
        Vector3 accelerationCmd = PureProNav(RelativePosition, RelativeVelocity, Gain);// + Vector3.up * 9.8f * gain/2;
        accelerationCmd += -Physics.gravity;
        return accelerationCmd;

    }

    Vector3 PureProNav(Vector3 relPos, Vector3 relVel, float gain)
    {
        Vector3 LoSRate = Vector3.Cross(relPos, relVel) / (relPos.magnitude * relPos.magnitude);

        float speedRatio = relVel.magnitude / m_velocity.magnitude;

        Vector3 accelerationCmd = -gain * speedRatio * Vector3.Cross(m_velocity, LoSRate);

        return accelerationCmd;
    }

}
