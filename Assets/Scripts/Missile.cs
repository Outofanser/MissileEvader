using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using Unity.VisualScripting;
using Unity.VisualScripting.FullSerializer;
using UnityEngine;

public class Missile : MonoBehaviour
{
    private GameObject m_target;
    private Rigidbody m_body;
    [SerializeField]
    private GameObject m_missileCOM;
    [SerializeField]
    private float m_launchSpeed = 100;
    private bool m_armed;
    private bool m_exploded;


    [SerializeField]
    private ParticleSystem explosionParticle;
    [SerializeField]
    private AudioClip explosionSound;
    private AudioSource missileAudio;
    private IEnumerator thrustLooper;
    [SerializeField]
    private AudioClip thrustSound;

    private AerodynamicsModel m_AeroModel;
    private AutoPilotController m_AutoPilot;
    public GameObject Target
    {
        get
        {
            return m_target;
        }
        set
        {
            m_target = value;
        }
    }

    void Awake()
    {
        m_body = GetComponent<Rigidbody>();
        missileAudio = GetComponent<AudioSource>();
        m_AeroModel = GetComponent<AerodynamicsModel>();
        m_AutoPilot = GetComponent<AutoPilotController>();
    }

    void Start()
    {
        
        m_body.centerOfMass = m_missileCOM.transform.localPosition;
        m_body.AddForce(Vector3.up * m_launchSpeed, ForceMode.VelocityChange);
        
        thrustLooper = LoopAudio(1f);
        missileAudio.clip = thrustSound;

        m_AutoPilot.Target = m_target;

        StartCoroutine(thrustLooper);

        explosionParticle.Stop();

    }

    void Update()
    {
        float tgo = m_AutoPilot.Tgo;

        if (tgo < 10 && tgo > 0 && m_AutoPilot.IsInPursuit)
        {
            m_armed = true;
        }
        if (tgo < 0 && m_armed)
        {
            Explode();
        }
    }

    void FixedUpdate()
    {
        Vector3 control = m_AutoPilot.GetControl();

        Vector3 torque = m_AeroModel.GenerateControlTorque(control);

        Vector3 aeroForces = m_AeroModel.GenerateAeroForces();
        m_body.AddForce(aeroForces);

        if (torque.magnitude > 0.001)
        {
            m_body.AddTorque(torque);
        }
    }


    private void Explode()
    {
        if (!m_exploded)
        {
            Damage();
            m_exploded = true;
            StopCoroutine(thrustLooper);
            explosionParticle.Play();
            missileAudio.Stop();
            missileAudio.PlayOneShot(explosionSound, 0.2f);
            Destroy(gameObject.transform.GetChild(0).gameObject);
            StartCoroutine(Explosion());
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Player"))
        {
            Explode();
        }
        else if (collision.gameObject.CompareTag("Ground"))
        {
            Explode();
        }
    }

    IEnumerator Explosion()
    {
        yield return new WaitForSeconds(2f);
        Destroy(gameObject);
    }

    private void Damage()
    {
        float distance = m_AutoPilot.RelativePosition.magnitude;

        float damage = 300f / distance;

        if (damage < 10f)
        {
            return;
        }

        m_target.GetComponent<PlayerController>().health -= Mathf.Min(damage, 80f);
        Debug.Log("Player hit! Health is now " + m_target.GetComponent<PlayerController>().health);
    }

    IEnumerator LoopAudio(float waitTime)
    {
        while (true)
        {
            float mywait = Mathf.Max(0.1f, Mathf.Min(1f, m_AutoPilot.RelativePosition.magnitude / 1000f)) * waitTime;
            missileAudio.time = 0f;
            missileAudio.Play();
            yield return new WaitForSeconds(mywait);
        }

    }


}
