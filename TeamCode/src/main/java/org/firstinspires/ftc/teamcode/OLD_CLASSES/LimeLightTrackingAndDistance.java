package org.firstinspires.ftc.teamcode.OLD_CLASSES;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class LimeLightTrackingAndDistance {

    double TA;
    double TX;
    double TY;
    boolean isValid;

    final double kP = 0.031, kI = 0.001, kD = 0.001;
    final double kF = 0.02;

    DcMotorEx turretMotor;
    PIDController controller = new PIDController(kP, kI, kD);

    final double SCALEFORDISTANCE = 172.3;
    final double POWERFORDISTANCE = -0.561;

    final double TERMONEFORVELOCITY = 724.441581;
    final double TERMTWOFORVELOCITY = 3.690488;
    final double TERMTHREEFORVELOCITY = 0.002708;
    final double TERMFOURFORVELOCITY = 0.000006;



    double LIMELIGHTHEIGHTFROMGROUNDINCHES = 12.25;
    double LIMELIGHTMOUNTEDANGLEDEGREES = 19.0;
    double GOALHEIGHTINCHES = 29.5;




    public LimeLightTrackingAndDistance(DcMotorEx turretMotor) {
        this.turretMotor = turretMotor;
        turretMotor.setTargetPositionTolerance(0);
        turretMotor.setPositionPIDFCoefficients(15.5);
    }


    public void setLLResult(LLResult llResult) {
        this.TA = llResult.getTa();
        this.TX = llResult.getTx();
        this.TY = llResult.getTy();
        this.isValid = llResult.isValid();
    }


    public void trackAprilTagWithPID () {
        controller.setPID(kP,kI,kD);
        double pid = controller.calculate(TX,0);
        double power = pid + kF;
        if (TX < 5 && TX > -5 && isValid) {
            turretMotor.setPower(0);
        } else {
            turretMotor.setPower(power);
        }

    }

    public void trackAprilTagWithPosition() {
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition() + (int) TX);
        turretMotor.setPower(1);
    }


//    IN PROGRESS TESTING
    public double distanceToTargetTwo() {
        return (GOALHEIGHTINCHES - LIMELIGHTHEIGHTFROMGROUNDINCHES) / Math.tan(Math.toRadians(LIMELIGHTMOUNTEDANGLEDEGREES + TY));
    }



    public double distanceToTarget() {
        return SCALEFORDISTANCE * Math.pow(TA, POWERFORDISTANCE);
    }

    public int calculateRPMForShooter() {
        double distance = distanceToTarget();
        double a = TERMTWOFORVELOCITY * distance;
        double b = TERMTHREEFORVELOCITY * Math.pow(distance, 2);
        double c = TERMFOURFORVELOCITY * Math.pow(distance, 3);
        double motorVelocity = TERMONEFORVELOCITY + a - b - c;
        return (int) Math.ceil(motorVelocity);
    }
}
