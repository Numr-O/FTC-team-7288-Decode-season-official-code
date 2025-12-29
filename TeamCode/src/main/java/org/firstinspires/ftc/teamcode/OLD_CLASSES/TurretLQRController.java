package org.firstinspires.ftc.teamcode.OLD_CLASSES;

public class TurretLQRController {

    private final double KOmega = 0.0028134;
    private final double KItegral = 0.02460078;
    private final double maxVoltage = 12;

    private double integralError = 0.0;
    public TurretLQRController() {}


    // @Param: targetvelocity rad/s @Param: currentvelocity rad/s @Return: motor voltage
    public double updateLQR(double targetVelocity, double currentVelocity) {
        double error = currentVelocity - targetVelocity;
        integralError += error;

        double voltage = -KOmega* error  -KItegral * integralError;

        voltage = clamp(voltage, -maxVoltage, maxVoltage);

        return voltage;
    }

    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max , value));
    }

}
