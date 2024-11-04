using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    [SerializeField] Transform[] povs;
    [SerializeField] float speed;
    [SerializeField] Transform plane;

    private int index = 0;
    private Vector3 target;

    private void Update()
    {
        if(Input.GetKeyDown(KeyCode.Alpha1)) index = 0;
        else if(Input.GetKeyDown(KeyCode.Alpha2)) index = 1;
        else if(Input.GetKeyDown(KeyCode.Alpha3)) index = 2;
        else if(Input.GetKeyDown(KeyCode.Alpha4)) index = 3;
        
        target = povs[index].position;
    }


    private void FixedUpdate()
    {
        transform.position = Vector3.MoveTowards(transform.position, target, speed * Time.deltaTime);
        //transform.rotation = Quaternion.Lerp(transform.rotation, plane.rotation, speed * Time.deltaTime);

        //make camera point at plane regardless of position
        if(index != 2)
            transform.LookAt(plane);
        else
            transform.rotation = Quaternion.Lerp(transform.rotation, plane.rotation, speed * Time.deltaTime);

        //clamp height above 6
        //if(transform.position.y < 6) transform.position = new Vector3(transform.position.x, 6, transform.position.z);

    }
}
