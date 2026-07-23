using UnityEngine;

public class MissileAeroModel : AerodynamicsModel
{
    [SerializeField]
    private float m_finArea = 0.1f;
    [SerializeField]
    private float m_finMaxDeflect_deg = 40f;
    [SerializeField]
    private float m_tailDistance = 2.5f;
    [SerializeField]
    private float m_missileRadius = 0.3f;
    [SerializeField]
    private float m_wingArea = 1f; // override default
    public override float LiftArea { get { return m_wingArea; } }
    public override float LateralArea { get { return m_wingArea; } }
    public override float MaxPitchDeflection { get { return m_finMaxDeflect_deg; } }
    public override float MaxYawDeflection { get { return m_finMaxDeflect_deg; } }
    public override float MaxRollDeflection { get { return m_finMaxDeflect_deg; } }
    public override float MaxPitchMoment { get { return 4f * DynPressure * m_finArea * m_tailDistance; } }
    public override float MaxYawMoment { get { return 4f * DynPressure * m_finArea * m_tailDistance; } }
    public override float MaxRollMoment { get { return 4f * DynPressure * m_finArea * m_missileRadius; } }
    public override float TrimAngle { get { return 0f; } }
    public override float CenterOfLift { get{ return 0f; } }

/*     override public Vector3 CalculateLocalAttitudeMoment(Vector3 attitudeControl)
        {

            if (attitudeControl.magnitude > 1)
            {
                attitudeControl = attitudeControl.normalized;
            }



            float liftCoefficient = 2f * Mathf.PI * m_finMaxDeflect_deg * Mathf.Deg2Rad; //* Mathf.Deg2Rad;

            float maxTorque = 4 * DynPressure * liftCoefficient * m_finArea * m_tailDistance;
            float maxRollTorque = 4 * DynPressure * liftCoefficient * m_finArea * m_missileRadius;
            Vector3 attitudeMoment_local = Vector3.zero;
            attitudeMoment_local.x = attitudeControl.x * maxTorque;
            attitudeMoment_local.y = attitudeControl.y * maxTorque;
            attitudeMoment_local.z = attitudeControl.z * maxRollTorque;

            Debug.Log("old moment: " + attitudeMoment_local);

            return attitudeMoment_local; // in local frame

        } */
}
