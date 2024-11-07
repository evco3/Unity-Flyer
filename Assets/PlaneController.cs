using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class PlaneController : MonoBehaviour
{

    //public GameObject elevator;
    //public GameObject aileronL;
    //public GameObject aileronR;
    //public GameObject rudder;


    public float throttleIncrement = 0.1f;

    public float maxThrust = 100f;
    public float responsiveness = 10f;

    public float liftPower = 4f;

    public float inducedDrag = 0.04f; 
    public float flapLift = 50f;
    public float flapAOA = 10f;  

    //general flight inputs
    private float throttle;
    private float pitch;   
    private float yaw;
    private float roll;

    private float AOA;
    private float AOAyaw;

    //stall variables
    private bool isStalling = false;
    private bool onRunway = false;
    public float stallAOA = 15f;   // Critical angle of attack in degrees
    private float stallSpeed = 50f; // Minimum speed to stall    


    
    //control surface variables
    bool aileronLeft = false;
    bool aileronRight = false;
    bool elevatorUp = false;
    bool elevatorDown = false;
    bool rudderLeft = false;
    bool rudderRight = false;
    bool airbrakeDeployed = false;
    bool gearDeployed = false;
    bool flapsDeployed = false;

    //local velocity vecotrs
    private Vector3 Velocity;
    private Vector3 localVEL;
    private Vector3 localANGVEL;
    private Vector3 localGs;
    private Vector3 lastVEl;

    Vector3 controlInput;

    //deffine directinoal drag coeff based on unity curve class where input is speed and output is drag

    [Header("Drag Curves")]
    [SerializeField] private AnimationCurve dragRight;
    [SerializeField] private AnimationCurve dragLeft;
    [SerializeField] private AnimationCurve dragUp;
    [SerializeField] private AnimationCurve dragDown;
    [SerializeField] private AnimationCurve dragForward;
    [SerializeField] private AnimationCurve dragBack;

    [SerializeField] private AnimationCurve liftCurve;
    [SerializeField] private AnimationCurve rudderAOAcurve;

    [SerializeField] private Vector3 turnspeed;
    [SerializeField] private Vector3 turnAccel;
    [SerializeField] private AnimationCurve SteeringCurve;




    private float responseModifier{
        get{
                return (rb.mass/10f) * responsiveness;
        }
    }
    
    Rigidbody rb;
    [SerializeField] TextMeshProUGUI HUD;
    [SerializeField] Transform prop;

    public static Vector3 Scale6(Vector3 val, float posx, float negx, float posy, float negy, float posz, float negz){
        Vector3 result = val;

        if(val.x > 0f) result.x *= posx;
        else if(val.x < 0f) result.x *= negx;

        if(val.y > 0f) result.y *= posy;
        else if(val.y < 0f) result.y *= negy;

        if(val.z > 0f) result.z *= posz;
        else if(val.z < 0f) result.z *= negz;

        return result;
    }
    
    // Start is called before the first frame update
    private void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void CalculateState(float dt){
        var invRotation  = Quaternion.Inverse(transform.rotation);
        Velocity = rb.velocity;
        localVEL = invRotation * Velocity;
        localANGVEL = invRotation * rb.angularVelocity;

    }

    void CalculateAOA(){
        if(localVEL.magnitude < 0.1f){
            AOA = 0f;
            AOAyaw = 0f;
            return;
        }
        AOA = Mathf.Atan2(-localVEL.y, localVEL.z);
        AOAyaw = Mathf.Atan2(localVEL.x, localVEL.z);

    }

    void CalcGForces(float dt){
        var invRotation = Quaternion.Inverse(transform.rotation);
        var localAcc = (rb.velocity - lastVEl) / dt;
        lastVEl = rb.velocity;
        localGs = invRotation * localAcc;
    }

    void OnCollisionEnter(Collision collision){
    if (collision.gameObject.CompareTag("Runway"))
    {
        onRunway = true;
        Debug.Log("Plane has landed on the runway!");
    }
    }

    void OnCollisionExit(Collision collision){
    if (collision.gameObject.CompareTag("Runway"))
    {
        onRunway = false;
        Debug.Log("Plane has left the runway!");
    }
    }

    void CheckForStall() {
        bool speedStall = localVEL.z < stallSpeed;
        bool angleStall = Mathf.Abs(AOA) > stallAOA;


        //allow plane not to stall on runway
        if(onRunway == false){
            isStalling = speedStall || angleStall;
        }else{
            isStalling = false;
        }
    }

    void StallRecoveryMechanic() {
    if (isStalling) {
        //slowly apply a rotation force to the plane to simulate a stall recovery. weaken force as plane begins to point towards the gorund
        float recoveryForce = Mathf.Clamp01(1f - (Vector3.Angle(transform.up, Vector3.up) / 180f));

        // Apply recovey force to plane using forceMode Force
        rb.AddTorque(transform.right * recoveryForce * 1.1f, ForceMode.Force);

        print("recoveryForce: " + recoveryForce);
        print("recovery force:" + recoveryForce);
        // Apply a small force forward to simulate air flowing past wings again
        rb.AddRelativeForce(Vector3.forward, ForceMode.Acceleration);
    }
}

    
    //DRAG CALCULATIONS//----------------------------------------------
    private void updateDrag(){
        var drag = Vector3.zero;
        if(localVEL.magnitude > 5f){
            var lv  = localVEL;


            float airbrakeDrag = airbrakeDeployed ? 0.5f : 0f;
            float gearDrag = gearDeployed ? 0.5f : 0f;
            float flapsDrag = flapsDeployed ? 0.5f : 0f;

            /*var DragCO = Scale6(lv.normalized, dragRight.Evaluate(Mathf.Abs(lv.x)),
                                            dragLeft.Evaluate(Mathf.Abs(lv.x)),
                                            dragUp.Evaluate(Mathf.Abs(lv.y)),
                                            dragDown.Evaluate(Mathf.Abs(lv.y)),
                                            dragForward.Evaluate(Mathf.Abs(lv.z)) + airbrakeDrag + gearDrag + flapsDrag,
                                            dragBack.Evaluate(Mathf.Abs(lv.z)));
            */

            //split up drag into each dimensionn
            float dragX = dragRight.Evaluate(Mathf.Abs(lv.x));
            float dragY = dragUp.Evaluate(Mathf.Abs(lv.y));
            float dragZ = dragForward.Evaluate(Mathf.Abs(lv.z)) + airbrakeDrag + gearDrag + flapsDrag;

            float stallDragReduction = isStalling ? 0.6f : 1f;


            //combine drag into a single vector
            Vector3 localDrag = new Vector3(
            dragX * lv.x * Mathf.Abs(lv.x) * 3,
            dragY * lv.y * Mathf.Abs(lv.y) * 2 * stallDragReduction,
            dragZ * lv.z * Mathf.Abs(lv.z)
            );


            //transform drag into world space
            Vector3 worldDrag = transform.TransformDirection(-localDrag);
            rb.AddForce(worldDrag);

            //print each component of drag
            //print("DragCo.magnitude: " + DragCO.magnitude);
           // drag = DragCO.magnitude * -lv.normalized * lv.sqrMagnitude;


            Debug.DrawLine(transform.position, transform.position + worldDrag, Color.red);

            //local velocity
            Vector3 relativeLocalVel = transform.TransformDirection(localVEL.normalized) * 5f;
            Debug.DrawLine(transform.position, transform.position + relativeLocalVel, Color.blue);


        }else{
            drag = Vector3.zero;
        }

        //output speed drag and thrust at the same time
        //print("Speed: " + localVEL.magnitude + " Drag: " + drag.magnitude + " Thrust: " + throttle * maxThrust);

    }


    ///LIFT CALCULATIONS//----------------------------------------------
    Vector3 CalculateLift(float AOA, Vector3 rightAxis, float liftPower, AnimationCurve aoaCurve){
       var liftVelocity = Vector3.ProjectOnPlane(localVEL, rightAxis);
       var v2 = liftVelocity.sqrMagnitude;

       float stallLiftReduction = isStalling ? 0.2f : 1f;

       var liftCoeff = aoaCurve.Evaluate(AOA * Mathf.Rad2Deg);
       var liftForce = v2 * liftCoeff * liftPower;

       var liftDir = Vector3.Cross(liftVelocity.normalized , rightAxis);
       var lift = liftDir * liftForce * stallLiftReduction;

        var dragForce = liftCoeff * liftCoeff * this.inducedDrag;
        var dragDir = -liftVelocity.normalized;
        var inducedDrag = dragDir * dragForce * v2;

        return lift + inducedDrag;
    }

    private void updateLift(){
        if(localVEL.magnitude < 1f) return;

        float flapLift = flapsDeployed ? this.flapLift : 0f;
        float flapAOA = flapsDeployed ? this.flapAOA : 0f;

        var liftForce = CalculateLift(AOA + (flapAOA*Mathf.Deg2Rad), Vector3.right, liftPower, liftCurve);

        //draw lift vector in up direction relative to plane
        Vector3 relativeLift = transform.TransformDirection(liftForce.normalized) * 5f;
        Debug.DrawLine(transform.position, transform.position + relativeLift, Color.green);

        rb.AddRelativeForce(liftForce);

        //var yawForce = CalculateLift(AOAyaw, Vector3.up, rudderPower, rudderAOAcurve);
        //rb.AddRelativeForce(yawForce);
    }


    //STEERING CALCULATIONS//----------------------------------------------
    float CalculateSteering(float dt, float angularVelocity, float targetVelocity, float acc){
        var error = targetVelocity - angularVelocity;
        var accel = acc*dt;
        return Mathf.Clamp(error, -accel,accel);
    }

    private void updateSteering(float dt){
        var speed = Mathf.Max(0, localVEL.z);
        var steering = SteeringCurve.Evaluate(speed);


        var target = Vector3.Scale(controlInput, turnspeed * steering);
        var av = localANGVEL * Mathf.Rad2Deg;

        var correction = new Vector3(CalculateSteering(dt, av.x, target.x, turnAccel.x),
                                      CalculateSteering(dt, av.y, target.y, turnAccel.y),
                                      CalculateSteering(dt, av.z, target.z, turnAccel.z));

        
        //print("localVEl:" + localVEL + " steering force: " + correction + " target: " + target + " av: " + av);

        rb.AddRelativeTorque(correction * Mathf.Deg2Rad, ForceMode.VelocityChange);
    }

    //HUD
    private void updateHUD(){
        if (HUD != null) {
            if (isStalling) {
                HUD.text = "";
                HUD.color = Color.red;
                HUD.text = "STALL";
            } else {
                HUD.color = Color.white;
                HUD.text = "Throttle: " + throttle.ToString("F0");
                HUD.text += "\nAirSpeed: " + localVEL.magnitude.ToString("F2") + " m/s";
                HUD.text += "\nAltitude: " + (transform.position.y / 2).ToString("F0") + " m";
            }
        }
    }  

    //INPUTS
    private void HandleInputs(){
        roll = Input.GetAxis("Roll");
        pitch = Input.GetAxis("Pitch");
        yaw = Input.GetAxis("Yaw");

        controlInput = new Vector3(pitch, yaw, -roll);

        if(Input.GetKey(KeyCode.Space)) throttle += throttleIncrement;
        else if(Input.GetKey(KeyCode.LeftShift)) throttle -= throttleIncrement;
        throttle = Mathf.Clamp(throttle, 0f, 100f);
    }

    // Update is called once per frame
    private void Update()
    {
        HandleInputs();
        updateHUD();
    }

    private void FixedUpdate(){

        float dt = Time.fixedDeltaTime;

        CalculateState(dt);
        CalculateAOA();
        CalcGForces(dt);
        CheckForStall();

        updateDrag();
        updateLift();
        updateSteering(dt);

        StallRecoveryMechanic();

        //rotate properller around local x axis
        prop.Rotate(-transform.forward, throttle/2, Space.World);

        //print("Rigidbody Rotation: " + rb.rotation.eulerAngles);
        //print("Velocity: " + rb.velocity);


        //print(localVEL);


        rb.AddRelativeForce(Vector3.forward * throttle * maxThrust);
    }    
}
