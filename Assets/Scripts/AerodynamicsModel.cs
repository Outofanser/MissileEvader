
using System.Runtime.InteropServices.WindowsRuntime;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Timeline;

abstract public class AerodynamicsModel : MonoBehaviour
{
    private const float c_rho = 1.293f;

    protected Rigidbody m_body;
    [SerializeField]
    protected Vector3 m_velocity;
    [SerializeField]
    protected float m_stallAoA = 12f;
    public float DynPressure
    {
        get
        {
            return 0.5f * c_rho * Mathf.Pow(m_velocity.magnitude, 2);
        }
    }
    protected Vector3 m_control;
    public abstract float WingArea { get; }

    void Awake()
    {
        m_body = GetComponent<Rigidbody>();
        m_velocity = Vector3.zero;
    }

    void Update()
    {
        m_velocity = m_body.linearVelocity;
    }

    void FixedUpdate()
    {

        Vector3 torque_local = CalculateLocalAttitudeMoment(m_control);
        Vector3 aeroForces = CalculateAeroForces();

        m_body.AddForce(aeroForces);

        if (torque_local.magnitude > 0.001)
        {
            m_body.AddRelativeTorque(torque_local);
        }
    }

    virtual public Vector3 CalculateAeroForces()
    {
        Vector3 body2velAngle = Vector3.Cross(m_body.transform.forward, m_velocity);
        Vector3 liftDirection = Vector3.Cross(m_velocity, body2velAngle).normalized;

        float attackAngle = Vector3.Angle(m_body.transform.forward, m_velocity);

        float liftCoefficient = 2f * Mathf.PI * attackAngle * Mathf.Deg2Rad;
        float dragCoefficient = 1f * Mathf.Pow(attackAngle * Mathf.Deg2Rad, 2f);

        if (attackAngle > m_stallAoA) // Stalling condition
        {
            liftCoefficient = Mathf.Sin(2f * attackAngle * Mathf.Deg2Rad);
            dragCoefficient = 1f - Mathf.Cos(2f * attackAngle * Mathf.Deg2Rad);
        }


        Vector3 liftForce = liftDirection * liftCoefficient * DynPressure * WingArea;
        Vector3 dragForce = -m_velocity.normalized * dragCoefficient * DynPressure * WingArea;

        return liftForce + dragForce; // in world frame
    }

    abstract public Vector3 CalculateLocalAttitudeMoment(Vector3 attitudeControl);

    virtual public void SetAttitudeControl(Vector3 control)
    {
        m_control = control; // Could add slew rates
    }

}
