package org.firstinspires.ftc.teamcode.OLD_CLASSES;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDriveConsentric {


    public double botHeading;

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    public BNO055IMU imu;


    public MecanumDriveConsentric(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, BNO055IMU imu) {
        this.frontLeftMotor = frontLeftMotor;
        this.backLeftMotor = backLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backRightMotor = backRightMotor;
        this.imu = imu;
    }




    public void controlerDrive(double Y, double X, double RX, double imuAngle,double headingOffset) {


        botHeading = -imu.getAngularOrientation().firstAngle - headingOffset;

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

        frontLeftMotor.setPower(frontLeftP);
        backLeftMotor.setPower(backLeftP);
        frontRightMotor.setPower(frontRightP);
        backRightMotor.setPower(backRightP);


    }




}