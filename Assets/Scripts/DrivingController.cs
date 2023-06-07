using System;
using System.Collections;
using System.Collections.Generic;
using System.Net.Sockets;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements;

public class DrivingController : MonoBehaviour
{
    [Header("Movement Settings")]
    [Tooltip("Top speed attainable going forward")]
    public float TopSpeed;

    [Tooltip("How fast the moped reaches top speed")]
    public float Acceleration;

    [Tooltip("Top speed attainable going backwards")]
    public float ReverseTopSpeed;

    [Tooltip("How fast the moped reaches top speed going backwards")]
    public float ReverseAcceleration;

    [Tooltip("How quickly moped starts accelerating from 0. Higher number = accelerates faster"), Range(0.2f, 1f)]
    public float AccelerationCurve;

    [Tooltip("How quickly the moped slows when braking")]
    public float Braking;

    [Tooltip("How quickly the moped will reach a stop when no inputs are made")]
    public float CoastingDrag;

    [Tooltip("The amount of side-to-side friction"), Range(0f, 1f)]
    public float Grip;

    [Tooltip("How tightly the moped can turn left or right")]
    public float Steer;

    [Tooltip("Additional gravity for when the moped is in the air.")]
    public float AddedGravity;

    public Rigidbody Rigidbody { get; private set; }
    public InputData Input { get; private set; }
    public float AirPercent { get; private set; }
    public float GroundPercent { get; private set; }

    [Header("Moped Physics")]
    [Tooltip("The transform to determine the position of the mopeds mass")]
    public Transform CenterOfMass;

    [Range(0.0f, 20.0f), Tooltip("Coefficient used to reorient the moped in the air. The higher the number, the faster the moped will readjust itself along the horizontal plane.")]
    public float AirborneReorientationCoefficient = 3.0f;

    [Header("Drifting")]
    [Range(0.01f, 1.0f), Tooltip("The grip value when drifting.")]
    public float DriftGrip = 0.4f;
    [Range(0.0f, 10.0f), Tooltip("Additional steer when the kart is drifting.")]
    public float DriftAdditionalSteer = 5.0f;
    [Range(1.0f, 30.0f), Tooltip("The higher the angle, the easier it is to regain full grip.")]
    public float MinAngleToFinishDrift = 10.0f;
    [Range(0.01f, 0.99f), Tooltip("Mininum speed percentage to switch back to full grip.")]
    public float MinSpeedPercentToFinishDrift = 0.5f;
    [Range(1.0f, 20.0f), Tooltip("The higher the value, the easier it is to control the drift steering.")]
    public float DriftControl = 10.0f;
    [Range(0.0f, 20.0f), Tooltip("The lower the value, the longer the drift will last without trying to control it by steering.")]
    public float DriftDampening = 10.0f;

    [Header("VFX")]

    [Tooltip("VFX that will be placed on the wheels when drifting")]
    public ParticleSystem DriftSparkVFX;
    [Tooltip("Offset to displace the VFX to the side"), Range(0f, 2f)]
    public float DriftSparkHorizontalOffset;
    [Tooltip("Angle to rotate the VFX"), Range(0f, 90f)]
    public float DriftSparkRotation = 17f;
    [Tooltip("VFX that will be placed on the wheels when drifting")]
    public GameObject DriftTrailPrefab;
    [Tooltip("Offset to displace the trail up or down to ensure they are above the ground"), Range(-0.1f, 0.1f)]
    public float DriftTrailVerticalOffset;

    //todo - think about adding exhaust to the moped and having hot air/smoke like vfx come out of it

    [Header("Physical Wheels")]
    [Tooltip("Physical representation of the moped's wheels")]
    public WheelCollider FrontWheel;
    public WheelCollider BackWheel;

    [Tooltip("Which layers the wheels can detect")]
    public LayerMask GroundLayers = Physics.DefaultRaycastLayers;

    //input sources that can control the moped
    IInput[] m_Inputs;

    //constant values for detecting lack of input/movement
    const float k_NullInput = 0.01f;
    const float k_NullSpeed = 0.01f;

    Vector3 m_VerticalReference = Vector3.up;

    //Drift Parameters
    public bool WantsToDrift { get; private set; } = false;
    public bool IsDrifting { get; private set; } = false;
    float m_CurrentGrip = 1.0f;
    float m_DriftTurningPower = 0.0f;
    float m_PreviousGroundPercent = 1.0f;

    //todo - list of trail instances & spark instances

    bool m_CanMove = true;
    Quaternion m_LastValidRotation;
    Vector3 m_LastValidPosition;
    Vector3 m_LastCollisionNormal;
    bool m_HasCollision;
    bool m_InAir = false;

    float FrontSlipOutput;
    float BackSlipOutput;

    public void SetCanMove(bool move) => m_CanMove = move;

    //todo - activate drift VFX & Orientation

    private void Awake()
    {
        Rigidbody = GetComponent<Rigidbody>();
        m_Inputs = GetComponents<IInput>();

        //add sparks & trails to wheel

        //add smoke to exhaust
    }

    private void FixedUpdate() 
    {
        //get input
        GetInput();
        //handle moped driving/turning/braking
        HandleMoped();

        //HandleMopedExtras();
            //checking for horn

        //ensure wheels have a correct amount of slip
        CalculateSlip();
    }

    private void GetInput()
    {
        //reset input
        Input = new InputData();
        WantsToDrift = false;

        //gather input from all sources
        for (int i = 0; i < m_Inputs.Length; i++)
        {
            Input = m_Inputs[i].GenerateInput();
            WantsToDrift = Input.Brake && Vector3.Dot(Rigidbody.velocity, transform.forward) > 0.0f;
        }
    }

    private void HandleMoped()
    {
        int groundedCount = 0;
        if (FrontWheel.isGrounded && FrontWheel.GetGroundHit(out WheelHit hit))
            groundedCount++;
        if (BackWheel.isGrounded && BackWheel.GetGroundHit(out hit))
            groundedCount++;

        GroundPercent = (float)groundedCount / 2.0f;
        AirPercent = 1 - GroundPercent;

        if (m_CanMove)
        {
            MoveMoped(Input.Accelerate, Input.Brake, Input.TurnInput);
        }

        if (AirPercent >= 1)
        {
            Rigidbody.velocity += Physics.gravity * Time.fixedDeltaTime * AddedGravity;
        }
        m_PreviousGroundPercent = GroundPercent;
    }

    public void Reset()
    {
        Vector3 euler = transform.rotation.eulerAngles;
        euler.x = euler.z = 0f;
        transform.rotation = Quaternion.Euler(euler);
    }

    public float LocalSpeed()
    {
        if (m_CanMove)
        {
            float dot = Vector3.Dot(transform.forward, Rigidbody.velocity);
            if (Mathf.Abs(dot) > 0.1f)
            {
                float speed = Rigidbody.velocity.magnitude;
                //if dot is negative we are going backwards otherwise forwards
                return dot < 0 ? -(speed / ReverseTopSpeed) : (speed / TopSpeed);
            }
            return 0f;
        }
        else
        {
            //player inputting when they can't move (maybe engine revving sound could do nicely here)
            return Input.Accelerate ? 1.0f : 0.0f;
        }
    }


    //Taken from Unity tutorial for 4 wheeled kart movement (might need to add 4 wheel colliders to the moped and just have the visual be 2 wheeled
        //also might need to adjust this code for it to work properly
        //breaking down each part of it and understanding it is vital
    private void MoveMoped(bool accelerate, bool brake, float turnInput)
    {
        //setup values to calculate movement

        //get acceleration input
        float accelInput = (accelerate ? 1.0f : 0.0f) - (brake ? 1.0f : 0.0f);

        //manual acceleration curve coefficient scalar
        //need to understand what purpose this serves
        float accelerationCurveCoeff = 5f;
        Vector3 localVel = transform.InverseTransformVector(Rigidbody.velocity);

        bool accelDirectionIsFwd = accelInput >= 0;
        bool localVelDirectionIsFwd = localVel.z >= 0;

        //set max speed and accel
        float maxSpeed = localVelDirectionIsFwd ? TopSpeed : ReverseTopSpeed;
        float accelPower = accelDirectionIsFwd ? Acceleration : ReverseAcceleration;

        //get current speed & acceleration
        float currentSpeed = Rigidbody.velocity.magnitude;
        float accelRampT = currentSpeed / maxSpeed;

        //acceleration curve + coefficient
        float multipliedAccelerationCurve = AccelerationCurve * accelerationCurveCoeff;
        //ramp up acceleration
        float accelRamp = Mathf.Lerp(multipliedAccelerationCurve, 1, accelRampT * accelRampT);

        bool isBraking = (localVelDirectionIsFwd && brake) || (!localVelDirectionIsFwd && accelerate);

        //calculate final acceleration
        float finalAccelPower = isBraking ? Braking : accelPower;
        float finalAcceleration = finalAccelPower * accelRamp;

        //handle turning
        float turningPower = IsDrifting ? m_DriftTurningPower : turnInput * Steer;

        Quaternion turnAngle = Quaternion.AngleAxis(turningPower, transform.up);
        Vector3 fwd = turnAngle * transform.forward;
        Vector3 movement = fwd * accelInput * finalAcceleration * ((m_HasCollision || GroundPercent > 0.0f) ? 1.0f : 0.0f);

        //forward movement
        bool wasOverMaxSpeed = currentSpeed >= maxSpeed;
        if (wasOverMaxSpeed && !isBraking)
            movement *= 0.0f;

        Vector3 newVelocity = Rigidbody.velocity + movement * Time.fixedDeltaTime;
        newVelocity.y = Rigidbody.velocity.y;

        //clamp max speed on ground
        if (GroundPercent > 0.0f && !wasOverMaxSpeed)
            newVelocity = Vector3.ClampMagnitude(newVelocity, maxSpeed);

        if (Mathf.Abs(accelInput) < k_NullInput && GroundPercent > 0.0f)
            newVelocity = Vector3.MoveTowards(newVelocity, new Vector3(0f, Rigidbody.velocity.y, 0f), Time.fixedDeltaTime * CoastingDrag);

        Rigidbody.velocity = newVelocity;

        //Drift
        if (GroundPercent > 0.0f)
        {
            if (m_InAir)
            {
                m_InAir = false;
                //Instantiate Jump VFX
            }

            //manual angular velocity coefficients
            float angularVelocitySteering = 0.4f;
            float angularVelocitySmoothSpeed = 20f;

            //turning is reversed if going backwards
            if (!localVelDirectionIsFwd && !accelDirectionIsFwd)
                angularVelocitySteering *= -1.0f;

            var angularVel = Rigidbody.angularVelocity;

            //move y angular velocity towards target
            angularVel.y = Mathf.MoveTowards(angularVel.y, turningPower * angularVelocitySteering, Time.fixedDeltaTime * angularVelocitySmoothSpeed);

            //apply angular velocity
            Rigidbody.angularVelocity = angularVel;

            //rotate rigidbody velocity to generate velocity redirection
            float velocitySteering = 25f;

            //If moped lands with forward not in the velocity direction, we start the drift
            if (GroundPercent >= 0.0f && m_PreviousGroundPercent < 0.1f)
            {
                Vector3 flattenVelocity = Vector3.ProjectOnPlane(Rigidbody.velocity, m_VerticalReference).normalized;
                if (Vector3.Dot(flattenVelocity, transform.forward * Mathf.Sign(accelInput)) < Mathf.Cos(MinAngleToFinishDrift * Mathf.Deg2Rad))
                {
                    IsDrifting = true;
                    m_CurrentGrip = DriftGrip;
                    m_DriftTurningPower = 0.0f;
                }
            }

            //Manage the drift
            if (!IsDrifting)
            {
                if ((WantsToDrift || isBraking) && currentSpeed > maxSpeed * MinSpeedPercentToFinishDrift)
                {
                    IsDrifting = true;
                    m_DriftTurningPower = turningPower + (Mathf.Sign(turningPower) * DriftAdditionalSteer);
                    m_CurrentGrip = DriftGrip;

                    //ActivateDriftVFX
                }
            }

            if (IsDrifting)
            {
                float turnInputAbs = Mathf.Abs(turnInput);
                if (turnInputAbs < k_NullInput)
                    m_DriftTurningPower = Mathf.MoveTowards(m_DriftTurningPower, 0.0f, Mathf.Clamp01(DriftDampening * Time.fixedDeltaTime));

                //update turning power based on input
                float driftMaxSteerValue = Steer + DriftAdditionalSteer;
                m_DriftTurningPower = Mathf.Clamp(m_DriftTurningPower + (turnInput + Mathf.Clamp01(DriftControl * Time.fixedDeltaTime)), -driftMaxSteerValue, driftMaxSteerValue);

                bool facingVelocity = Vector3.Dot(Rigidbody.velocity.normalized, transform.forward * Mathf.Sign(accelInput)) > Mathf.Cos(MinAngleToFinishDrift * Mathf.Deg2Rad);

                bool canEndDrift = true;
                if (isBraking)
                    canEndDrift = false;
                else if (!facingVelocity)
                    canEndDrift = false;
                else if (turnInputAbs >= k_NullInput && currentSpeed > maxSpeed * MinSpeedPercentToFinishDrift)
                    canEndDrift = false;

                if (canEndDrift || currentSpeed < k_NullSpeed)
                {
                    //if no input and car aligned with speed direction stop drift
                    IsDrifting = false;
                    m_CurrentGrip = Grip;
                }
            }

            //rotate velocity based on current steer value
            Rigidbody.velocity = Quaternion.AngleAxis(turningPower * Mathf.Sign(localVel.z) * velocitySteering * m_CurrentGrip * Time.fixedDeltaTime, transform.up) * Rigidbody.velocity;
        }
        else
        {
            m_InAir = true;
        }

        bool validPosition = false;
        //raycast on the layers the player can drive on
        if (Physics.Raycast(transform.position + (transform.up * 0.1f), -transform.up, out RaycastHit hit, 3.0f, 1 << 6)) //Layer 6 : Drivable
        {
            Vector3 lerpVector = (m_HasCollision && m_LastCollisionNormal.y > hit.normal.y) ? m_LastCollisionNormal : hit.normal;
            m_VerticalReference = Vector3.Slerp(m_VerticalReference, lerpVector, Mathf.Clamp01(AirborneReorientationCoefficient * Time.fixedDeltaTime * (GroundPercent > 0.0f ? 10.0f : 1.0f))); //Blend faster on ground
        }
        else
        {
            Vector3 lerpVector = (m_HasCollision && m_LastCollisionNormal.y > 0.0f) ? m_LastCollisionNormal : Vector3.up;
            m_VerticalReference = Vector3.Slerp(m_VerticalReference, lerpVector, Mathf.Clamp01(AirborneReorientationCoefficient * Time.fixedDeltaTime));
        }

        validPosition = GroundPercent > 0.7f && !m_HasCollision && Vector3.Dot(m_VerticalReference, Vector3.up) > 0.9f;

        // Airborne / Half on ground management
        if (GroundPercent < 0.7f)
        {
            Rigidbody.angularVelocity = new Vector3(0.0f, Rigidbody.angularVelocity.y * 0.98f, 0.0f);
            Vector3 finalOrientationDirection = Vector3.ProjectOnPlane(transform.forward, m_VerticalReference);
            finalOrientationDirection.Normalize();
            if (finalOrientationDirection.sqrMagnitude > 0.0f) 
            {
                Rigidbody.MoveRotation(Quaternion.Lerp(Rigidbody.rotation, Quaternion.LookRotation(finalOrientationDirection, m_VerticalReference), Mathf.Clamp01(AirborneReorientationCoefficient * Time.fixedDeltaTime)));
            }
        }
        else if (validPosition)
        {
            m_LastValidPosition = transform.position;
            m_LastValidRotation.eulerAngles = new Vector3(0.0f, transform.rotation.y, 0.0f);
        }

        //Activate Drift VFX(IsDrifting && GroundPercent > 0.0f);
    }

    //detecting collisions
    private void OnCollisionEnter(Collision collision) => m_HasCollision = true;
    private void OnCollisionExit(Collision collision) => m_HasCollision = false;

    //gathering information from collisions
    void OnCollisionStay(Collision collision)
    {
        m_HasCollision = true;
        m_LastCollisionNormal = Vector3.zero;
        float dot = -1.0f;

        foreach (var contact in collision.contacts)
        {
            if (Vector3.Dot(contact.normal, Vector3.up) > dot)
                m_LastCollisionNormal = contact.normal;
        }
    }

    //For Debugging Purposes vvv

    private void CalculateSlip()
    {
        WheelFrictionCurve frontWFC = FrontWheel.sidewaysFriction;
        WheelFrictionCurve backWFC = BackWheel.sidewaysFriction;

        WheelHit hit = new WheelHit();
        if (FrontWheel.GetGroundHit(out hit))
        {
            FrontSlipOutput = hit.sidewaysSlip / frontWFC.extremumSlip;
        }

        WheelHit hitTwo = new WheelHit();
        if (BackWheel.GetGroundHit(out hitTwo))
        {
            BackSlipOutput = hitTwo.sidewaysSlip / backWFC.extremumSlip;
        }
    }

    private void OnGUI()
    {
        GUI.Label(
            new Rect(
                5, //x, left offset
                Screen.height - 150, //y, bottom offset
                300f, //width
                50f //height
                ),
            "Front: " + FrontSlipOutput + " | Back: " + BackSlipOutput,
            GUI.skin.textArea);
    }
}
