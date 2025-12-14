package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class GYROCorrection {

    double xPos;
    double yPos;
    double Kmt = 2.57;
    public double trueErrorCorrection;
    public double rotationError;
    public double translationErrorAngle;
    LLResult llResult;
    SparkFunOTOS sparkFunOTOS;
    DcMotorEx turretMotor;



    public GYROCorrection(SparkFunOTOS sparkFunOTOS, DcMotorEx turretMotor) {
        this.sparkFunOTOS = sparkFunOTOS;
        this.turretMotor = turretMotor;
    }


    public void setLLResultForGYRO(LLResult llResult) {
        this.llResult = llResult;
    }


    public void trackWithGYRO (double initialDistance, double imuAngle) {
        xPos = sparkFunOTOS.getPosition().x;
        yPos = sparkFunOTOS.getPosition().y;

        translationErrorAngle = Math.toDegrees(Math.atan(xPos / (initialDistance - yPos)));

        rotationError = Math.toDegrees(imuAngle);
        double limeLightError = llResult.getTx();

        trueErrorCorrection = (rotationError - translationErrorAngle) - limeLightError;

        double motorOutput = Kmt * trueErrorCorrection;

        turretMotor.setTargetPosition((int) motorOutput);
        turretMotor.setPower(1);
    }



}
