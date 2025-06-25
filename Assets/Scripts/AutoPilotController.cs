
using Unity.VisualScripting;
using UnityEngine;

public class AutoPilotController : MonoBehaviour
{
    //fields
    private GameObject m_target; 
    [SerializeField]
    private float m_timeConstant = 0.8f;
    [SerializeField]
    private float m_angleOfAttackLimit_deg;
    [Header("Vertical Attitude PID")]
    [SerializeField] private PIDController m_vertPIDController;
    [Header("Horizontal Attitude PID")]
    [SerializeField] private PIDController m_horzPIDController;
    [Header("Roll Attitude PID")]
    [SerializeField] private PIDController m_rollPIDController;
    private AerodynamicsModel m_AeroModel;
    private Vector3 m_velocity;
    private Vector3 m_accelerationCmd;
    private Rigidbody m_body;
    [field: SerializeField]
    public bool IsInPursuit { get; private set; } = false;

    //properties
    public Vector3 RelativePosition { get; private set; }
    public Vector3 RelativeVelocity { get; private set; }
    [field: SerializeField]
    public float NavGain { get; set; } = 3f;
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
        //m_pidController = GetComponent<PIDController>();
        m_vertPIDController = new PIDController();
        m_horzPIDController = new PIDController();
        m_rollPIDController = new PIDController();

        m_AeroModel = GetComponent<AerodynamicsModel>();
        m_body = GetComponent<Rigidbody>();
    }

    void Start()
    {
        m_angleOfAttackLimit_deg = m_AeroModel.StallAoA_deg - 2f;
    }

    void Update()
    {
        Vector3 targetPosition = m_target.GetComponent<PlayerController>().transform.position;
        Vector3 targetVelocity = m_target.transform.forward * m_target.GetComponent<PlayerController>().airSpeed;
        RelativePosition = targetPosition - transform.position;
        RelativeVelocity = targetVelocity - m_body.linearVelocity;

        m_velocity = m_body.linearVelocity;

        Tgo = -RelativePosition.magnitude * RelativePosition.magnitude / Vector3.Dot(RelativePosition, RelativeVelocity);

        // determine acceleration command Strategy
        if (IsInPursuit)
        {
            m_accelerationCmd = CommandPursuit();
        }
        else
        {
            m_accelerationCmd = CommandGravityTurn();
        }

        // Leave gravity turn when we are stable
        if (!IsInPursuit && (m_accelerationCmd.normalized.y > 0 || m_accelerationCmd.normalized.y < -0.95f))
        {
            IsInPursuit = true;
        }

    }

    public Vector3 ComputeAutoPilotControl()
    {
        float area = m_AeroModel.WingArea;
        float dynamicPressure = m_AeroModel.DynPressure;

        // Lift Force ~= 2pi * AoA * dynP * area
        float desiredAngleOfAttack = m_accelerationCmd.magnitude * m_body.mass / area / dynamicPressure / (2 * Mathf.PI);
        float attackLimited = Mathf.Min(desiredAngleOfAttack * Mathf.Rad2Deg, m_angleOfAttackLimit_deg);

        // Compute the desired forward direction to achieve the AoA
        Vector3 turnAxis = Vector3.Cross(m_velocity, m_accelerationCmd).normalized;
        Vector3 desiredForward = Quaternion.AngleAxis(attackLimited, turnAxis) * m_velocity.normalized;

        // Calculate the error angle from this new desiredForward
        Vector3 misalignmentAxis = -Vector3.Cross(desiredForward, m_body.transform.forward);
        float misalignmentAngle = Mathf.Asin(misalignmentAxis.magnitude);

        Quaternion worldToBodyRotation = Quaternion.Inverse(transform.rotation);

        Vector3 alignmentError_local = worldToBodyRotation * misalignmentAxis.normalized * misalignmentAngle;  // z component should be zero      
        
        // get the error rate to calculate the control for this time step
        Vector3 alignmentErrorRate_local = alignmentError_local / m_timeConstant; // Slew control to moderate the error rate
        Vector3 attitudeControl = Vector3.zero;
        attitudeControl.x = m_vertPIDController.PID(alignmentErrorRate_local.x/2f/Mathf.PI);
        attitudeControl.y = m_horzPIDController.PID(alignmentErrorRate_local.y/2f/Mathf.PI);

        // skip doing roll control for now I don't need it -- need a better method
        //Vector3 angularVelocity_local = worldToBodyRotation * m_body.angularVelocity;
        //attitudeControl.z = m_rollPIDController.PID(-angularVelocity_local.z/2f/Mathf.PI);

        // could also add a cascade PID control: sets a desired rate from the error in outer PID and an inner PID controls the error rate

        return attitudeControl;
    }

    Vector3 CommandGravityTurn()
    {
        // this method is not great. It scales with the control gains so we will spin out if the attitude gain is too high
        Vector3 rotationDir = Vector3.Cross(m_velocity, RelativePosition).normalized;
        Vector3 accelerationCmd = Vector3.Cross(rotationDir, m_velocity).normalized * 9.8f;
        //accelerationCmd += -Physics.gravity;
        return accelerationCmd;
        
    }

    Vector3 CommandPursuit()
    {
        Vector3 accelerationCmd = PureProNav(RelativePosition, RelativeVelocity, NavGain);// + Vector3.up * 9.8f * gain/2;
        accelerationCmd += -Physics.gravity;
        return accelerationCmd;

    }

    Vector3 PureProNav(Vector3 relPos, Vector3 relVel, float gain)
    {
        Vector3 LOSRate = Vector3.Cross(relPos, relVel) / (relPos.magnitude * relPos.magnitude);

        float speedRatio = relVel.magnitude / m_velocity.magnitude;

        Vector3 accelerationCmd = -gain * speedRatio * Vector3.Cross(m_velocity, LOSRate);

        return accelerationCmd;
    }

}
