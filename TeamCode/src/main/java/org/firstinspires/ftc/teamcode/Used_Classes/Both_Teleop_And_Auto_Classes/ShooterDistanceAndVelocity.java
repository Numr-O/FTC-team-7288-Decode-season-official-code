package org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes;

public class ShooterDistanceAndVelocity {
    double SCALE_FOR_DISTANCE = 172.3;
    double POWER_FOR_DISTANCE = -0.561;



    public double distanceToTarget(double TA) {
        return SCALE_FOR_DISTANCE * Math.pow(TA, POWER_FOR_DISTANCE);
    }


    public double distanceToSpeed(double distance) {
        return 1.67279 * distance + 889.73139;
    }

}
