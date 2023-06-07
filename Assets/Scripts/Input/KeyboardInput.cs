using UnityEngine;

public class KeyboardInput : BaseInput
{
    public string TurnInputName = "Horizontal";
    public string AccelerateButtonName = "Accelerate";
    public string BrakeButtonName = "Brake";
    public string DriftButtonName = "Drift";
    public string HonkButtonName = "Honk";

    public override InputData GenerateInput()
    {
        return new InputData
        {
            Accelerate = Input.GetButton(AccelerateButtonName),
            Brake = Input.GetButton(BrakeButtonName),
            Drift = Input.GetButton(DriftButtonName),
            Honk = Input.GetButton(HonkButtonName),
            TurnInput = Input.GetAxis("Horizontal")
        };
    }
}
