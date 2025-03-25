using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class PlaneController : MonoBehaviour
{

    [SerializeField] float MaxThrust;
    [SerializeField] float liftPower;
    [SerializeField] float inducedDrag;

    [SerializeField] Vector3 PositiveDragValues;
    [SerializeField] Vector3 NegativeDragValues;

    [SerializeField] AnimationCurve Lift;

    [SerializeField] Vector3 TurnSpeed;
    [SerializeField] Vector3 TurnAcceleration;
    [SerializeField] AnimationCurve SteeringCurve;




    Rigidbody rb;
    Vector3 localVelocity;
    Vector3 lastVelocity;
    Vector3 localGForce;
    Vector3 localAngularVelocity;

    float angleOfAttack;
    float angleOfAttackYaw;

    float currentPitchAmount;
    float currentYawAmount;
    float currnetRollAmount;
    float currentThrustAmount;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        CalculateState(Time.deltaTime);
        CalculateAngleOfAttack();
        CalculateGForce();

        UpdateThrust();
        UpdateDrag();
        UpdateLift();
        UpdateSteering();
    }

    void UpdateThrust() 
    {
        rb.AddRelativeForce(currentThrustAmount * MaxThrust * Vector3.forward);
    }

    void UpdateDrag() 
    {
        Vector3 coefficient = Scale6(localVelocity.normalized, PositiveDragValues, NegativeDragValues);
        Vector3 drag = coefficient.magnitude * localVelocity.sqrMagnitude * -localVelocity.normalized;
    }

    void UpdateLift() 
    {
        if (localVelocity.sqrMagnitude < 1f) return;

        Vector3 liftForce = CalculateLift(angleOfAttack,liftPower);
        Vector3 yawForce = CalculateLift(angleOfAttackYaw,liftPower);

        rb.AddRelativeForce(liftForce);
        rb.AddRelativeForce(yawForce);
    }

    void UpdateSteering() 
    {
        float speed = MathF.Max(0, localVelocity.z);
        float steeringPower = SteeringCurve.Evaluate(speed);

        Vector3 targetAV = Vector3.Scale(new Vector3(currentPitchAmount, currnetRollAmount, currentYawAmount), TurnSpeed * steeringPower);
        Vector3 av = localAngularVelocity * Mathf.Rad2Deg;

        Vector3 correction = new Vector3
            (
                CalculateSteering(av.x, targetAV.x, TurnAcceleration.x * steeringPower),
                CalculateSteering(av.y, targetAV.y, TurnAcceleration.y * steeringPower),
                CalculateSteering(av.z, targetAV.z, TurnAcceleration.z * steeringPower)
            );

        rb.AddTorque(correction * Mathf.Deg2Rad, ForceMode.VelocityChange);
    }

    private void CalculateState(float dt)
    {
        Quaternion invRotation = Quaternion.Inverse(rb.rotation);
        localVelocity = invRotation * rb.velocity;
        localAngularVelocity = invRotation * rb.angularVelocity;
    }

    private void CalculateAngleOfAttack()
    {
        if (localVelocity.sqrMagnitude < 0.1f)
        {
            angleOfAttack = 0;
            angleOfAttackYaw = 0;
            return;
        }

        angleOfAttack = Mathf.Atan2(-localVelocity.y, localVelocity.z);
        angleOfAttackYaw = Mathf.Atan2(localVelocity.x, localVelocity.z);
    }

    private void CalculateGForce() 
    {
        Quaternion invRotation = Quaternion.Inverse(rb.rotation);
        Vector3 acceleration = (rb.velocity - lastVelocity) / Time.deltaTime;
        localGForce = invRotation * acceleration;
        lastVelocity = rb.velocity;
    }

    Vector3 CalculateLift(float angleOfAttack, float liftPower) 
    {
        Vector3 liftVelocity = Vector3.ProjectOnPlane(localVelocity, Vector3.right);
        float v2 = liftVelocity.sqrMagnitude;

        float liftCoefficient = Lift.Evaluate(angleOfAttack * Mathf.Rad2Deg);
        float liftForce = v2 * liftCoefficient * liftPower;

        Vector3 liftDirection = Vector3.Cross(liftVelocity.normalized, Vector3.right);
        Vector3 lift = liftDirection * liftForce;

        float dragForce = liftCoefficient * liftCoefficient * this.inducedDrag;
        Vector3 dragDirection = -liftVelocity.normalized;
        Vector3 inducedDrag = dragDirection * liftVelocity.sqrMagnitude * dragForce;

        return lift + inducedDrag;
    }

    float CalculateSteering(float targetVelocity, float angularVelocity, float acceleration) 
    {
        float error = targetVelocity - angularVelocity;
        var accel = acceleration * Time.deltaTime;
        return Mathf.Clamp(error, -accel, accel);
    }

    public void SetCurrentPitch(InputAction.CallbackContext context) 
    {
        currentPitchAmount = context.ReadValue<float>();
    }

    public void SetCurrentYaw(InputAction.CallbackContext context)
    {
        currentYawAmount = context.ReadValue<float>();
    }

    public void SetCurrentRoll(InputAction.CallbackContext context)
    {
        currnetRollAmount = context.ReadValue<float>();
    }

    public void SetCurrentThrust(InputAction.CallbackContext context)
    {
        currentThrustAmount = context.ReadValue<float>();
    }

    //helper function

    public static Vector3 Scale6(Vector3 value, Vector3 positiveValues, Vector3 negativeValues) 
    {
        Vector3 res = value;

        res.x *= res.x > 0 ? positiveValues.x : negativeValues.x;
        res.y *= res.y > 0 ? positiveValues.y : negativeValues.y;
        res.z *= res.z > 0 ? positiveValues.z : negativeValues.z;

        return res;
    }

}
