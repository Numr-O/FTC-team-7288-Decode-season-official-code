package org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {


    public BNO055IMU imu;
    public RevIMU revIMU;
    public SparkFunOTOS sparkFunOTOS;

    public Limelight3A limelight;

    public Servo ledLight;


    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotorEx turretMotor;
    public DcMotor intakeMotor;


    public Servo intakeServoLeft;
    public Servo intakeServoRight;
    public Servo indexerServo;


    public DcMotorEx shooterMotorTop;
    public DcMotorEx shooterMotorBottom;


    public ColorRangeSensor colorPosA; //Indexer Servo 0.9
    public ColorRangeSensor colorPosB; //Indexer Servo 0.5
    public ColorRangeSensor colorPosC; //Indexer Servo 0.1

    public SparkFunOTOS.Pose2D pose2D;

    HardwareMap hwmap;

    public void init(HardwareMap ghwmap) {

        hwmap = ghwmap;

        frontLeftMotor = hwmap.get(DcMotor.class, "fl");
        backLeftMotor = hwmap.get(DcMotor.class, "bl");
        frontRightMotor = hwmap.get(DcMotor.class, "fr");
        backRightMotor = hwmap.get(DcMotor.class, "br");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        turretMotor = hwmap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intakeMotor = hwmap.get(DcMotor.class, "test");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intakeServoLeft = hwmap.get(Servo.class, "intakeL");
        intakeServoRight = hwmap.get(Servo.class, "intakeR");


        indexerServo = hwmap.get(Servo.class, "indexer");


        ledLight = hwmap.get(Servo.class, "led");

        shooterMotorTop = hwmap.get(DcMotorEx.class, "smTop");
        shooterMotorBottom = hwmap.get(DcMotorEx.class, "smBottom");

        shooterMotorTop.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotorBottom.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

//        shooterMotorTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shooterMotorBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        shooterMotorTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterMotorBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hwmap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        colorPosA = hwmap.get(ColorRangeSensor.class, "colorPosA");
        colorPosB = hwmap.get(ColorRangeSensor.class, "colorPosB");
        colorPosC = hwmap.get(ColorRangeSensor.class, "colorPosC");


        limelight = hwmap.get(Limelight3A.class, "LimeLight");


        pose2D = new SparkFunOTOS.Pose2D(0,0,-90);
        sparkFunOTOS = hwmap.get(SparkFunOTOS.class, "otos");
//        sparkFunOTOS.setOffset(pose2D);
        sparkFunOTOS.setAngularScalar(0.997);
        sparkFunOTOS.setLinearUnit(DistanceUnit.CM);
        sparkFunOTOS.setLinearScalar(1.006);

    }





}
