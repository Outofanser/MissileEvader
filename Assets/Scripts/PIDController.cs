using UnityEngine;

public class PIDController : MonoBehaviour
{
    private Vector3 m_errorLast = Vector3.zero;
    private Vector3 m_errorAccumulator = Vector3.zero;
    [SerializeField]
    private float pGain = 1f;
    [SerializeField]
    private float iGain = 0.001f;
    [SerializeField]
    private float dGain = 0.2f;

    void Start()
    {

    }

    public Vector3 PID(Vector3 error)
    {
        Vector3 dError = (error - m_errorLast) / Time.fixedDeltaTime;
        Vector3 iError = m_errorAccumulator + error * Time.fixedDeltaTime;

        if (m_errorLast == Vector3.zero)
        {
            dError = Vector3.zero;
        }

        Vector3 PID = pGain * error + dGain * dError + iGain * iError;

        if (PID.magnitude > 1)
        {
            PID = PID.normalized;
        }

        m_errorLast = error;
        m_errorAccumulator += error;


        return PID;
    }
}
