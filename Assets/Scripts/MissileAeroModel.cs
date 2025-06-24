using UnityEngine;

public class MissileAeroModel : AerodynamicsModel
{
    [SerializeField]
    private float m_finArea = 0.1f;
    [SerializeField]
    private float m_finMaxDeflect_deg = 40f;
    [SerializeField]
    private float m_tailToCMDistance = 2.5f;
    [SerializeField]
    private float m_missileRadius = 0.3f;
    [SerializeField]
    private float m_wingArea = 1f; // override default
    public override float WingArea { get { return m_wingArea; } }

    override public Vector3 CalculateLocalAttitudeMoment(Vector3 attitudeControl)
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
}
