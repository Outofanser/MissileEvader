using System;
using UnityEngine;

[Serializable]
public class PIDController
{
    //private Vector3 m_errorLast = Vector3.zero;
    private float m_errorLast;
    //private Vector3 m_errorAccumulator = Vector3.zero;
    private float m_errorAccumulator;
    [SerializeField] private float pGain = 1f;
    [SerializeField] private float iGain = 0.001f;
    [SerializeField] private float dGain = 0.2f;
    public float PGain { get => pGain; set => pGain = value; }
    public float IGain { get => iGain; set => iGain = value; }
    public float DGain { get => dGain; set => dGain = value; }

    public PIDController() { }
    public PIDController(float p, float i, float d)
    {
        pGain = p;
        iGain = i;
        dGain = d;
    }
    public float PID(float error)
    {
        float dError = (error - m_errorLast) / Time.fixedDeltaTime;
        float iError = m_errorAccumulator + error * Time.fixedDeltaTime;

        if (m_errorLast == 0)
        {
            dError = 0;
        }

        float PID = pGain * error + dGain * dError + iGain * iError;

        if (PID > 1)
        {
            PID = 1;
        }
        else if (PID < -1)
        {
            PID = -1;
        }

        m_errorLast = error;
        m_errorAccumulator += error;

        return PID;
    }

    public void Reset()
    {
        m_errorLast = 0;
        m_errorAccumulator = 0;
    }
}
