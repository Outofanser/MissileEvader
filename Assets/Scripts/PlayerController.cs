using Microsoft.Win32.SafeHandles;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
    private Vector3 initPosition;
    private Quaternion initRotation;


    // Start is called before the first frame update
    void Start()
    {
        planeAudio = GetComponent<AudioSource>();
        initPosition = transform.position;
        initRotation = transform.rotation;
    }

    // Update is called once per frame
    void Update()
    {

        if (health < 0 && !destroyed)
        {
            destroyed = true;
            Explode();
        }

        if (!destroyed)
        {
            PlayerMovement();
            ApplyBoundary();
        }

    }

    void PlayerMovement()
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
    }

    void ApplyBoundary()
    {
        transform.position = new Vector3(MinMax(-500f, 500f, transform.position.x), MinMax(0f, 200f, transform.position.y), MinMax(-500f, 500f, transform.position.z));
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
    }

}
