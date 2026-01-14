package org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes;

import com.arcrobotics.ftclib.controller.PIDFController;

public class TurretPositionPIDController {

    static double kP = 0.01, kI = 0, kD = 0, kF = 0;
    PIDFController pidfController = new PIDFController(kP,kI,kD,kF);

    double Kmt = 2.57;




    public int updatePosition(double xPos, double yPos, double currentAngleError, double currentTXError, double initialDistance) {

        double translationErrorAngle = Math.toDegrees(Math.atan(xPos / (initialDistance - yPos)));

        double trueErrorCorrection = (currentAngleError + translationErrorAngle) - currentTXError;
        double desiredMotorPosition = Kmt * trueErrorCorrection;

        return (int) desiredMotorPosition;
    }


}
