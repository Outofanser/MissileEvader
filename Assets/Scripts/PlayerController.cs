using Microsoft.Win32.SafeHandles;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UIElements;

public class PlayerController : MonoBehaviour
{
    InputAction attitudeAction;
    InputAction yawAction;
    InputAction lookAction;
    float maxHorzLookAngle_deg = 170;
    float maxVertLookAngle_deg = 90;
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

        lookAction = InputSystem.actions.FindAction("FlightControl/Look");
        attitudeAction = InputSystem.actions.FindAction("FlightControl/Attitude");
        yawAction = InputSystem.actions.FindAction("FlightControl/Yaw");
    }

    // Update is called once per frame
    void Update()
    {

        ApplyCameraRotation();

        if (health < 0 && !destroyed)
        {
            destroyed = true;
            Explode();
        }

        if (!destroyed)
        {
            ApplyPlayerInput();
            ApplyBoundary();
            propeller.transform.Rotate(new Vector3(0, 0, 1000 * Time.deltaTime));
        }

    }

    void ApplyCameraRotation()
    {
        m_playerCam.transform.localRotation = m_cameraLookAngle;
        m_playerCam.transform.localPosition = m_cameraOffset;

        Vector2 lookInput = lookAction.ReadValue<Vector2>();

        m_playerCam.transform.RotateAround(transform.position, transform.up, maxHorzLookAngle_deg * lookInput.x);
        m_playerCam.transform.RotateAround(transform.position, transform.right, maxVertLookAngle_deg * lookInput.y);


        Vector3 attackAngleAxis = Vector3.Cross(transform.forward, m_body.linearVelocity.normalized);
        float attackAngle = Mathf.Asin(attackAngleAxis.magnitude);
        m_playerCam.transform.RotateAround(transform.position, attackAngleAxis, attackAngle*Mathf.Rad2Deg);

    }

    void ApplyPlayerInput()
    {
        Vector2 attitudeInput = attitudeAction.ReadValue<Vector2>();
        Vector3 playerAttitudeInput = Vector3.zero;

        playerAttitudeInput.x = attitudeInput.y;
        playerAttitudeInput.y = yawAction.ReadValue<float>();
        playerAttitudeInput.z = -attitudeInput.x;

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
