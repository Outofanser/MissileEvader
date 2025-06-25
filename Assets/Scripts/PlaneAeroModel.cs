using UnityEngine;

public class PlaneAeroModel : AerodynamicsModel
{
    [SerializeField] private float m_finArea = 0.1f;
    [SerializeField] private float m_finMaxDeflect_deg = 20f;
    [SerializeField] private float m_tailArea = 0.8f;
    [SerializeField] private float m_tailMaxDeflect_deg = 20f;
    [SerializeField] private float m_aerolonArea = 0.2f;
    [SerializeField] private float m_aerolonMaxDeflect_deg = 20f;
    [SerializeField] private float m_tailDistance = 2.5f;
    [SerializeField] private float m_aerolonDistance = 1f;
    [SerializeField] private float m_wingArea = 2f; // override default
    [SerializeField] private float m_bodyLongitudinalCrossSection = 2f;
    public override float WingArea { get { return m_wingArea; } }

    override public Vector3 CalculateLocalAttitudeMoment(Vector3 attitudeControl)
    {

        float pitchControlCoefficient = 2f * Mathf.PI * m_tailMaxDeflect_deg * Mathf.Deg2Rad;
        float yawControlCoefficient = 2f * Mathf.PI * m_finMaxDeflect_deg * Mathf.Deg2Rad;
        float rollControlCoefficient = 2f * Mathf.PI * m_aerolonMaxDeflect_deg * Mathf.Deg2Rad;

        float maxPitchTorque = 1 * DynPressure * m_tailArea * m_tailDistance;
        float maxYawTorque = 1 * DynPressure * m_finArea * m_tailDistance;
        float maxRollTorque = 2 * DynPressure * m_aerolonArea * m_aerolonDistance;

        float maxControlPitchTorque = maxPitchTorque * pitchControlCoefficient;
        float maxControlYawTorque = maxYawTorque * yawControlCoefficient;
        float maxControlRollTorque = maxRollTorque * rollControlCoefficient;

        Vector3 attitudeMoment_local = Vector3.zero;
        attitudeMoment_local.x = attitudeControl.x * maxControlPitchTorque;
        attitudeMoment_local.y = attitudeControl.y * maxControlYawTorque;
        attitudeMoment_local.z = attitudeControl.z * maxControlRollTorque;

        Vector3 attackAngleAxis = -Vector3.Cross(m_velocity.normalized, transform.forward);
        float attackAngle = Mathf.Asin(attackAngleAxis.magnitude);
        if (attackAngleAxis.magnitude < 0.0001) { attackAngle = 0; }
        Quaternion worldToLocalTransform = Quaternion.Inverse(transform.rotation);
        Vector3 attackAngleAxis_local = worldToLocalTransform * attackAngleAxis.normalized;

        Vector3 aeroMoment_local = Vector3.zero;
        aeroMoment_local.x = -2f * Mathf.PI * Mathf.Cos(attackAngle) * m_tailDistance * (worldToLocalTransform * m_body.angularVelocity).x / m_velocity.magnitude * maxPitchTorque;
        aeroMoment_local.y = -2f * Mathf.PI * Mathf.Cos(attackAngle) * m_tailDistance * (worldToLocalTransform * m_body.angularVelocity).y / m_velocity.magnitude * DynPressure * m_bodyLongitudinalCrossSection * m_tailDistance * 0.75f;
        //aeroMoment_local.z = 2f * Mathf.PI * attackAngle * attackAngleAxis_local.z * maxRollTorque; // zero no?
        //aeroMoment_local.z -= 0.1f * 2 * DynPressure * m_wingArea * m_aerolonDistance * (worldToLocalTransform * m_body.angularVelocity).z;
        // what is the proper way to account for aero moment?
        aeroMoment_local.z = -2f * Mathf.PI * Mathf.Cos(attackAngle) * 2f * DynPressure * m_wingArea * m_aerolonDistance * m_aerolonDistance * (worldToLocalTransform * m_body.angularVelocity).z / m_velocity.magnitude;

        return attitudeMoment_local + aeroMoment_local; // in local frame

    }
}
