using Microsoft.Win32.SafeHandles;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class PlayerController : MonoBehaviour
{
    public float pitchRate = 20; // deg/s
    public float rollRate = 30;
    public float yawRate = 10;
    public float airSpeed = 30; // m/s

    public float health = 100;
    private bool destroyed = false;

    // Make these private
    public Vector3 acceleration;

    public ParticleSystem explosionParticle;
    public AudioClip explosionSound;
    private AudioSource planeAudio;
    [SerializeField] private Camera m_playerCam;
    private Vector3 m_cameraOffset;
    private Quaternion m_cameraLookAngle;
    private Vector3 initPosition;
    private Quaternion initRotation;

    public GameObject propeller;

    private AerodynamicsModel m_AeroModel;
    private Rigidbody m_body;
    [SerializeField] private GameObject m_planeCOM;
    //public Vector3 CenterOfMass { get => m_body.centerOfMass; } 


    void Awake()
    {
        planeAudio = GetComponent<AudioSource>();
        m_AeroModel = GetComponent<AerodynamicsModel>();
        m_body = GetComponent<Rigidbody>();
        initPosition = transform.position;
        initRotation = transform.rotation;

    }

    void Start()
    {
        m_body.centerOfMass = m_planeCOM.transform.localPosition;
        m_body.AddForce(80f * transform.forward, ForceMode.VelocityChange);

        m_cameraOffset = m_playerCam.transform.localPosition;
        m_cameraLookAngle = m_playerCam.transform.localRotation;
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 velocityCoord_local = Quaternion.Inverse(transform.rotation) * m_body.linearVelocity;
        if (velocityCoord_local != Vector3.zero)
        {
            m_playerCam.transform.localPosition = Quaternion.LookRotation(velocityCoord_local) * m_cameraOffset;
            m_playerCam.transform.localRotation = Quaternion.LookRotation(velocityCoord_local) * m_cameraLookAngle;
        }
        if (health < 0 && !destroyed)
        {
            destroyed = true;
            Explode();
        }

        if (!destroyed)
        {
            //PlayerMovement();
            ApplyBoundary();
            propeller.transform.Rotate(new Vector3(0, 0, 1000 * Time.deltaTime));
        }

    }

    /*     void PlayerMovement()
        {
            // Move player forward in forward direction (not realistic physics)
            transform.Translate(Vector3.forward * airSpeed * Time.deltaTime);

            // get inputs to attitude controls
            Vector3 playerAttitudeInput = new Vector3(-Input.GetAxis("Pitch"), -Input.GetAxis("Yaw"), -Input.GetAxis("Roll"));
            // scale inputs to body moments (body rates)
            Vector3 rotationRate = Vector3.Scale(playerAttitudeInput, new Vector3(pitchRate, yawRate, rollRate));

            acceleration = Vector3.Cross(Vector3.forward * airSpeed, rotationRate); // may be useful information to use truth data with missile system

            // Rotate player via inputs
            transform.Rotate(rotationRate * Time.deltaTime);
        } */
    void FixedUpdate()
    {
        if (destroyed)
        {
            return;
        }

        Vector3 playerAttitudeInput = new Vector3(-Input.GetAxis("Pitch"), -Input.GetAxis("Yaw"), -Input.GetAxis("Roll"));

        // clamp the player pitch, we will add control to remove the clamp later
        playerAttitudeInput.x = MinMax(-0.3f, 0.3f, playerAttitudeInput.x);

        m_AeroModel.SetAttitudeControl(playerAttitudeInput);

    }

    void ApplyBoundary()
    {
        transform.position = new Vector3(MinMax(-5000f, 5000f, transform.position.x), MinMax(-10f, 2000f, transform.position.y), MinMax(-5000f, 5000f, transform.position.z));
    }

    float MinMax(float minVal, float maxVal, float compVal)
    {
        return Mathf.Max(minVal, Mathf.Min(compVal, maxVal));
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Ground"))
        {
            health -= 500;
            Debug.Log("Hit the ground!");
        }
    }

    private void Explode()
    {
        explosionParticle.Play();
        planeAudio.Stop();
        planeAudio.PlayOneShot(explosionSound, 0.2f);
        gameObject.transform.GetChild(0).gameObject.SetActive(false);
        StartCoroutine(Respawn());
    }

    private IEnumerator Respawn()
    {
        yield return new WaitForSeconds(3f);
        gameObject.transform.GetChild(0).gameObject.SetActive(true);
        gameObject.transform.position = initPosition;
        gameObject.transform.rotation = initRotation;
        destroyed = false;
        health = 100;
        m_body.linearVelocity = Vector3.zero;
        m_body.angularVelocity = Vector3.zero;
        m_body.AddForce(80f*transform.forward, ForceMode.VelocityChange);
    }

}
