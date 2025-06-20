using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpawnManager : MonoBehaviour
{
    public float waveNumber = 1;
    public float spawnRadius = 800;
    public float minSpawnRadius = 100;
    public GameObject missilePrefab;
    private GameObject player;


    void Start()
    {
        player = GameObject.FindGameObjectWithTag("Player");
    }

    void Update()
    {
        int numMissiles = GameObject.FindGameObjectsWithTag("Missile").Length;

        if (numMissiles <= 0)
        {
            for (int i = 0; i < waveNumber; i++)
            {
                GameObject missile = Instantiate(missilePrefab, GenerateSpawnPosition(), Quaternion.Euler(new Vector3(-90, 0, 0)));
                missile.GetComponent<Missile>().Target = player;
            }

            waveNumber++;

        }
    }
    
    Vector3 GenerateSpawnPosition ()
    {
        float xPos = 0;
        float zPos = 0;
        while (Mathf.Abs(xPos) < minSpawnRadius)
        {
            xPos = Random.Range(-spawnRadius, spawnRadius);
        }

        while (Mathf.Abs(zPos) < minSpawnRadius)
        {
            zPos = Random.Range(-spawnRadius, spawnRadius);
        }
        return new Vector3(xPos, 4f, zPos) + player.transform.position;
    }
}
