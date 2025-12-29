package org.firstinspires.ftc.teamcode.OLD_CLASSES;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class GYROCorrection {

    double xPos;
    double yPos;
    double Kmt = 2.57;
    public double trueErrorCorrection;
    public double rotationError;
    public double translationErrorAngle;
    public double positionControl;
    public double targetMotorPosition;
    LLResult llResult;
    SparkFunOTOS sparkFunOTOS;
    DcMotorEx turretMotor;

    PIDFCoefficients pidfCoefficients;
    PIDFController controller;

    double kP = 0,kI = 0,kD = 0,kF = 0;

    public GYROCorrection(SparkFunOTOS sparkFunOTOS, DcMotorEx turretMotor) {
        this.sparkFunOTOS = sparkFunOTOS;
        this.turretMotor = turretMotor;
        controller = new PIDFController(kP,kI,kD,kF);

    }



    public void setLLResultForGYRO(LLResult llResult) {
        this.llResult = llResult;
    }

    public void updatePIDF (double kP,double kI,double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void trackWithGYRO (double initialDistance, double imuAngle, double currentPosition) {
        controller.setPIDF(kP,kI,kD,kF);

        xPos = sparkFunOTOS.getPosition().x;
        yPos = sparkFunOTOS.getPosition().y;

        translationErrorAngle = Math.toDegrees(Math.atan(xPos / (initialDistance - yPos)));

        rotationError = Math.toDegrees(imuAngle);
        double limeLightError = llResult.getTx();

        trueErrorCorrection = (rotationError - translationErrorAngle) - limeLightError;

        targetMotorPosition = Kmt * trueErrorCorrection;

        positionControl = controller.calculate(currentPosition,targetMotorPosition);
        turretMotor.setPower(positionControl);

    }



}
