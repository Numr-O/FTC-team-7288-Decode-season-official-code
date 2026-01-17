package org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;

public class MecanumDrive {
    RobotHardware robotHardware = new RobotHardware();

    public void init(HardwareMap hardwareMap) {
        robotHardware.init(hardwareMap);
    }

    public void drive(double Y, double X, double RX,double botHeading) {


        double rotX = X * Math.cos(botHeading) - Y * Math.sin(botHeading);
        double rotY = X * Math.sin(botHeading) + Y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(Y) + Math.abs(X) + Math.abs(RX), 1);

        //Front Left Wheel Power
        double frontLeftP = (rotY + rotX + RX) / denominator;
        //Back Left Wheel Power
        double backLeftP = (rotY - rotX + RX) / denominator;
        //Front Right Wheel Power
        double frontRightP = (rotY - rotX - RX) / denominator;
        //Back Right Wheel Power
        double backRightP = (rotY + rotX - RX) / denominator;

        robotHardware.frontLeftMotor.setPower(frontLeftP);
        robotHardware.backLeftMotor.setPower(backLeftP);
        robotHardware.frontRightMotor.setPower(frontRightP);
        robotHardware.backRightMotor.setPower(backRightP);
    }

}
