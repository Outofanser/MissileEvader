
using System.Runtime.InteropServices.WindowsRuntime;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Timeline;

abstract public class AerodynamicsModel : MonoBehaviour
{
    private const float c_rho = 1.293f;

    protected Rigidbody m_body;
    [SerializeField] float m_speed;
    [SerializeField] float m_energy;
    [SerializeField] float m_attackAngle;
    [SerializeField] float m_sideSlipAngle;
    [SerializeField] protected Vector3 m_velocity;
    [SerializeField] protected Vector3 m_velocity_local;
    [SerializeField] protected float m_stallAoA_deg = 12f;
    public float StallAoA_deg { get => m_stallAoA_deg; set => m_stallAoA_deg = value; }
    public float DynPressure
    {
        get
        {
            return 0.5f * c_rho * Mathf.Pow(m_velocity.magnitude, 2);
        }
    }
    protected Vector3 m_control;
    public abstract float LiftArea { get; }
    public abstract float LateralArea { get; }
    public abstract float MaxPitchDeflection { get; }
    public abstract float MaxYawDeflection { get; }
    public abstract float MaxRollDeflection { get; }
    public abstract float MaxPitchMoment { get; }
    public abstract float MaxYawMoment { get; }
    public abstract float MaxRollMoment { get; }
    public abstract float TrimAngle { get; }
    public abstract float CenterOfLift { get; }
    public float AirSpeed { get { return m_speed; } }

    void Awake()
    {
        m_body = GetComponent<Rigidbody>();
        m_velocity = Vector3.zero;
    }

    void Update()
    {
        m_velocity = m_body.linearVelocity;
        m_speed = m_velocity.magnitude;
        m_energy = 0.5f * m_speed * m_speed + 9.8f * transform.position.y;

        m_velocity_local = Quaternion.Inverse(transform.rotation) * m_velocity;

        if (m_velocity_local.magnitude < 0.0000001)
        {
            m_attackAngle = 0f;
        }
        else
        {
            m_attackAngle = -Mathf.Asin(m_velocity_local.y / m_velocity_local.magnitude) * Mathf.Rad2Deg;
        }
        m_sideSlipAngle = -Mathf.Atan2(m_velocity_local.x, m_velocity_local.z) * Mathf.Rad2Deg;
    }

    void FixedUpdate()
    {

        Vector3 torque_local = CalculateLocalAttitudeMoment(m_control);
        Vector3 aeroForces = CalculateLocalAeroForces();

        //m_body.AddRelativeForce(aeroForces);

        Vector3 centerOfLift = new Vector3(0, 0, -CenterOfLift);
        m_body.AddForceAtPosition(transform.rotation * aeroForces, transform.TransformPoint(centerOfLift));

        if (torque_local.magnitude > 0.001)
        {
            m_body.AddRelativeTorque(torque_local);
        }
    }

    virtual public Vector3 CalculateLocalAeroForces()
    {
        float liftCoefficient = 2f * Mathf.PI * m_attackAngle * Mathf.Deg2Rad;
        float dragCoefficient = 1f * Mathf.Pow(m_attackAngle * Mathf.Deg2Rad, 2f);

        if (m_attackAngle > m_stallAoA_deg) // Stalling condition
        {
            liftCoefficient = Mathf.Sin(2f * m_attackAngle * Mathf.Deg2Rad);
            dragCoefficient = 1f - Mathf.Cos(2f * m_attackAngle * Mathf.Deg2Rad);
        }

        Vector3 liftForce = Quaternion.AngleAxis(m_attackAngle, Vector3.right) * (liftCoefficient * DynPressure * Vector3.up * LiftArea);
        Vector3 dragForce = -dragCoefficient * DynPressure * m_velocity_local.normalized * LiftArea;

        float lateralCoefficient = 2f * Mathf.PI * m_sideSlipAngle * Mathf.Deg2Rad;
        float lateralDragCoefficient = 1f * Mathf.Pow(m_sideSlipAngle * Mathf.Deg2Rad, 2f);

        if (m_sideSlipAngle > m_stallAoA_deg) // Stalling condition
        {
            lateralCoefficient = Mathf.Sin(2f * m_sideSlipAngle * Mathf.Deg2Rad);
            lateralDragCoefficient = 1f - Mathf.Cos(2f * m_sideSlipAngle * Mathf.Deg2Rad);
        }

        Vector3 lateralForce = Quaternion.AngleAxis(m_sideSlipAngle, Vector3.down) * (lateralCoefficient * DynPressure * Vector3.right * LateralArea);
        Vector3 lateralDragForce = -lateralDragCoefficient * DynPressure * m_velocity_local.normalized * LateralArea;

        return liftForce + dragForce + lateralForce + lateralDragForce;

    }


    virtual public Vector3 CalculateLocalAttitudeMoment(Vector3 attitudeControl)
    {

        float pitchDeflection = MaxPitchDeflection * attitudeControl.x - TrimAngle;
        float yawDeflection = MaxYawDeflection * attitudeControl.y;
        float rollDeflection = MaxRollDeflection * attitudeControl.z;

        float pitchAttackAngle = pitchDeflection;// + m_attackAngle;
        float yawAttackAngle = yawDeflection;// + m_sideSlipAngle;
        float rollAttackAngle = rollDeflection;

        float pitchControlCoefficient = 2f * Mathf.PI * pitchAttackAngle * Mathf.Deg2Rad;
        float yawControlCoefficient = 2f * Mathf.PI * yawAttackAngle * Mathf.Deg2Rad;
        float rollControlCoefficient = 2f * Mathf.PI * rollAttackAngle * Mathf.Deg2Rad;

        Vector3 attitudeMoment_local = Vector3.zero;

        attitudeMoment_local.x = MaxPitchMoment * pitchControlCoefficient;
        attitudeMoment_local.y = MaxYawMoment * yawControlCoefficient;
        attitudeMoment_local.z = MaxRollMoment * rollControlCoefficient;

        return attitudeMoment_local;
    }



    virtual public void SetAttitudeControl(Vector3 control)
    {
        m_control = control; // Could add slew rates
    }

}
