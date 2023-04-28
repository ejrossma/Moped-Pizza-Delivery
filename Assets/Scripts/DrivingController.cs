using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DrivingController : MonoBehaviour
{
    private const string HORIZONTAL = "Horizontal";
    private const string VERTICAL = "Vertical";

    private float horizontalInput;
    private float verticalInput;

    private float currentSteerAngle;
    private float currentBreakForce;
    private float currentLeanAngle;
    private bool isBreaking;

    private Rigidbody rb;

    private float frontSlipOutput;
    private float backSlipOutput;

    [SerializeField] private float motorForce;
    [SerializeField] private float breakForce;
    [SerializeField] private float maxSteeringAngle;
    [SerializeField] private float downForce;
    [SerializeField] private Vector3 centerOfMass;

    [SerializeField] private WheelCollider frontWheelCollider;
    [SerializeField] private Transform frontWheelTransform;
    [SerializeField] private WheelCollider backWheelCollider;
    [SerializeField] private Transform backWheelTransform;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = centerOfMass;
    }

    private void FixedUpdate() 
    {
        GetInput();
        HandleMotor();
        HandleSteering();
        //ApplyDownForce();
        CalculateSlip();
        //HandleBikeLean();
    }

    private void GetInput()
    {
        horizontalInput = Input.GetAxis(HORIZONTAL);
        verticalInput = Input.GetAxis(VERTICAL);
        isBreaking = Input.GetKey(KeyCode.Space);
    }

    private void HandleMotor()
    {
        backWheelCollider.motorTorque = verticalInput * motorForce;
        //only apply breakforce if the player isBreaking
        currentBreakForce = isBreaking ? breakForce : 0f;
        ApplyBreaking();
    }

    private void ApplyBreaking()
    {
        frontWheelCollider.brakeTorque = currentBreakForce;
        backWheelCollider.brakeTorque = currentBreakForce;
    }

    private void HandleSteering()
    {
        currentSteerAngle = maxSteeringAngle * horizontalInput;
        frontWheelCollider.steerAngle = currentSteerAngle;
    }

    private void ApplyDownForce()
    {
        //scale it with speed
        float downForceMultiplier = 1 - Mathf.Clamp01(rb.velocity.magnitude / 10f);
        Vector3 downForceVector = -transform.up * downForce * downForceMultiplier;
        rb.AddForce(downForceVector, ForceMode.Acceleration);
        WheelFrictionCurve sidewaysWFC = frontWheelCollider.sidewaysFriction;

        sidewaysWFC.extremumSlip = sidewaysWFC.extremumSlip * (1 + (downForce * downForceMultiplier) / rb.mass);
        sidewaysWFC.asymptoteSlip = sidewaysWFC.asymptoteSlip * (1 + (downForce * downForceMultiplier) / rb.mass);

        frontWheelCollider.sidewaysFriction = sidewaysWFC;
    }

    private void CalculateSlip()
    {
        WheelFrictionCurve frontWFC = frontWheelCollider.sidewaysFriction;
        WheelFrictionCurve backWFC = backWheelCollider.sidewaysFriction;

        WheelHit hit = new WheelHit();
        if (frontWheelCollider.GetGroundHit(out hit))
        {
            frontSlipOutput = hit.sidewaysSlip / frontWFC.extremumSlip;
        }

        WheelHit hitTwo = new WheelHit();
        if (backWheelCollider.GetGroundHit(out hitTwo))
        {
            backSlipOutput = hitTwo.sidewaysSlip / backWFC.extremumSlip;
        }
    }

    private void OnGUI()
    {
        GUI.Label(
            new Rect(
                5, //x, left offset
                Screen.height - 150, //y, bottom offset
                300f, //width
                150f //height
                ),
            "Front: " + frontSlipOutput + " | Back: " + backSlipOutput,
            GUI.skin.textArea);
    }
}
