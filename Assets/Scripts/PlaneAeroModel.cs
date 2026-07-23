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
    [SerializeField] private float m_aerolonDistance = 0.5f;
    [SerializeField] private float m_wingArea = 2f; // override default
    [SerializeField] private float m_bodyLongitudinalCrossSection = 2f;
    [SerializeField] private float m_trimAngle = 0f;
    [SerializeField] private float m_centerOfLift = 0.5f;
    public override float LiftArea { get { return m_wingArea; } }
    public override float LateralArea{ get { return m_bodyLongitudinalCrossSection; } }
    public override float MaxPitchDeflection{ get { return m_tailMaxDeflect_deg; } }
    public override float MaxYawDeflection{ get { return m_finMaxDeflect_deg; } }
    public override float MaxRollDeflection{ get { return m_aerolonMaxDeflect_deg; } }
    public override float MaxPitchMoment{ get { return 1f * DynPressure * m_tailArea * m_tailDistance; } }
    public override float MaxYawMoment{ get { return 1f * DynPressure * m_finArea * m_tailDistance; } }
    public override float MaxRollMoment{ get { return 1f * DynPressure * m_aerolonArea * m_aerolonDistance; } }
    public override float TrimAngle{ get { return m_trimAngle; } }
    public override float CenterOfLift { get{ return m_centerOfLift; } }
    

    override public Vector3 CalculateLocalAttitudeMoment(Vector3 attitudeControl)
    {

        Vector3 attitudeMoment_local = base.CalculateLocalAttitudeMoment(attitudeControl);

        Vector3 attackAngleAxis = -Vector3.Cross(m_velocity.normalized, transform.forward);
        float attackAngle = Mathf.Asin(attackAngleAxis.magnitude);
        if (attackAngleAxis.magnitude < 0.0001) { attackAngle = 0; }
        Quaternion worldToLocalTransform = Quaternion.Inverse(transform.rotation);
        //Vector3 attackAngleAxis_local = worldToLocalTransform * attackAngleAxis.normalized;

        Vector3 aeroMoment_local = Vector3.zero;
        aeroMoment_local.x = -2f * Mathf.PI * Mathf.Cos(attackAngle) * m_tailDistance * (worldToLocalTransform * m_body.angularVelocity).x / m_velocity.magnitude * MaxPitchMoment;
        aeroMoment_local.y = -2f * Mathf.PI * Mathf.Cos(attackAngle) * m_tailDistance * (worldToLocalTransform * m_body.angularVelocity).y / m_velocity.magnitude * DynPressure * m_bodyLongitudinalCrossSection * m_tailDistance * 0.75f;
        aeroMoment_local.z = -2f * Mathf.PI * Mathf.Cos(attackAngle) * 2f * DynPressure * m_wingArea * m_aerolonDistance * m_aerolonDistance * (worldToLocalTransform * m_body.angularVelocity).z / m_velocity.magnitude;

        return attitudeMoment_local + aeroMoment_local; // in local frame

    }
}
