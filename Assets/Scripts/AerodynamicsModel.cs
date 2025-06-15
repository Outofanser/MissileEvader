using System.Runtime.InteropServices.WindowsRuntime;
using UnityEngine;

public class AerodynamicsModel : MonoBehaviour
{

    private const float c_rho = 1.293f;
    [SerializeField]
    private float m_wingArea = 1f;
    [SerializeField]
    private float m_tailArea = 0.1f;
    [SerializeField]
    private float m_maxDeflect = 10f;
    [SerializeField]
    private float m_tail2CMDistance = 0.5f;
    private Rigidbody m_body;
    [SerializeField]
    private Vector3 m_velocity;
    private float m_dynPressure;
    public float DynPressure { get { return m_dynPressure; } private set { m_dynPressure = value; } }
    public float WingArea { get { return m_wingArea; } private set { m_wingArea = value; } }

    void Awake()
    {
        m_body = GetComponent<Rigidbody>();
        m_velocity = Vector3.zero;
    }

    void Update()
    {
        m_velocity = m_body.linearVelocity;
        m_dynPressure = 0.5f * c_rho * Mathf.Pow(m_velocity.magnitude, 2);
    }

    public Vector3 GenerateAeroForces()
    {
        Vector3 body2velAngle = Vector3.Cross(m_body.transform.forward, m_velocity);
        Vector3  lift_direction= Vector3.Cross(m_velocity, body2velAngle).normalized;

        float attackAngle = Vector3.Angle(m_body.transform.forward, m_velocity);

        float lift_coefficient = 2f * Mathf.PI * attackAngle * Mathf.Deg2Rad;
        float drag_coefficient = 1f * Mathf.Pow(attackAngle * Mathf.Deg2Rad, 2f);

        if (attackAngle > 12) // Stalling condition
        {
            lift_coefficient = Mathf.Sin(2f * attackAngle * Mathf.Deg2Rad);
            drag_coefficient = 1f - Mathf.Cos(2f * attackAngle * Mathf.Deg2Rad);
        }


        Vector3 lift_force = lift_direction * lift_coefficient * m_dynPressure * m_wingArea;
        Vector3 drag_force = -m_velocity.normalized * drag_coefficient * m_dynPressure * m_wingArea;

        return lift_force + drag_force;
    }

    public Vector3 GenerateControlTorque(Vector3 control)
    {


        float lift_coefficient = 2f * Mathf.PI * m_maxDeflect;

        float max_torque = m_dynPressure * lift_coefficient * m_tailArea * m_tail2CMDistance;
        Vector3 desired_torque = control * max_torque;

        return desired_torque;

    }

}
