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

    private Vector3 velocity = Vector3.zero;
    [SerializeField] float smoothTime = 0.1f;


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
        transform.position = Vector3.SmoothDamp(transform.position, target, ref velocity, smoothTime);
        //transform.position = Vector3.MoveTowards(transform.position, target, speed * Time.deltaTime);
        //transform.rotation = Quaternion.Lerp(transform.rotation, plane.rotation, speed * Time.deltaTime);

        //make camera point at plane regardless of position
        if(index != 2){
            // Look at the plane with a delayed rotation
            Quaternion lookRotation = Quaternion.LookRotation(plane.position - transform.position);
            transform.rotation = Quaternion.Slerp(transform.rotation, lookRotation, speed * Time.deltaTime);
        }
        else
            transform.rotation = Quaternion.Lerp(transform.rotation, plane.rotation, speed * Time.deltaTime);
    }
}
