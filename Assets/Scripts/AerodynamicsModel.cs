
using System.Runtime.InteropServices.WindowsRuntime;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Timeline;

public class AerodynamicsModel : MonoBehaviour
{
    private const float c_rho = 1.293f;
    [SerializeField]
    private float m_wingArea = 1f;
    [SerializeField]
    private float m_finArea = 0.1f;
    [SerializeField]
    private float m_finMaxDeflect_deg = 40f;
    [SerializeField]
    private float m_tailToCMDistance = 2.5f;
    [SerializeField]
    private float m_missileRadius = 0.3f;
    private Rigidbody m_body;
    [SerializeField]
    private Vector3 m_velocity;
    public float DynPressure
    {
        get
        {
            return 0.5f * c_rho * Mathf.Pow(m_velocity.magnitude, 2);
        }
    }
    private Vector3 m_control;
    public float WingArea { get { return m_wingArea; } private set { m_wingArea = value; } }

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

    public Vector3 CalculateAeroForces()
    {
        Vector3 body2velAngle = Vector3.Cross(m_body.transform.forward, m_velocity);
        Vector3 liftDirection = Vector3.Cross(m_velocity, body2velAngle).normalized;

        float attackAngle = Vector3.Angle(m_body.transform.forward, m_velocity);

        float liftCoefficient = 2f * Mathf.PI * attackAngle * Mathf.Deg2Rad;
        float dragCoefficient = 1f * Mathf.Pow(attackAngle * Mathf.Deg2Rad, 2f);

        if (attackAngle > 12) // Stalling condition
        {
            liftCoefficient = Mathf.Sin(2f * attackAngle * Mathf.Deg2Rad);
            dragCoefficient = 1f - Mathf.Cos(2f * attackAngle * Mathf.Deg2Rad);
        }


        Vector3 liftForce = liftDirection * liftCoefficient * DynPressure * m_wingArea;
        Vector3 dragForce = -m_velocity.normalized * dragCoefficient * DynPressure * m_wingArea;

        return liftForce + dragForce; // in world frame
    }

    public Vector3 CalculateLocalAttitudeMoment(Vector3 attitudeControl)
    {

        if (attitudeControl.magnitude > 1)
        {
            attitudeControl = attitudeControl.normalized;
        }

        float liftCoefficient = 2f * Mathf.PI * m_finMaxDeflect_deg * Mathf.Deg2Rad; //* Mathf.Deg2Rad;

        float maxTorque = 4 * DynPressure * liftCoefficient * m_finArea * m_tailToCMDistance;
        float maxRollTorque = 4 * DynPressure * liftCoefficient * m_finArea * m_missileRadius;
        Vector3 attitudeMoment_local = Vector3.zero;
        attitudeMoment_local.x = attitudeControl.x * maxTorque;
        attitudeMoment_local.y = attitudeControl.y * maxTorque;
        attitudeMoment_local.z = attitudeControl.z * maxRollTorque;

        return attitudeMoment_local; // in local frame

    }

    public void SetAttitudeControl(Vector3 control)
    {
        m_control = control; // Could add slew rates
    }

}
