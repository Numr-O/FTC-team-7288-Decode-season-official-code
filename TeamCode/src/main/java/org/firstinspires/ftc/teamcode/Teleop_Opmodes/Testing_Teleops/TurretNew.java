package org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;


@Configurable
public class TurretNew extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    MecanumDrive mecanumDrive = new MecanumDrive();
    File file;
    String[] fileContents;
    SimpleMatrix initialVector = new SimpleMatrix(2,1);
    SimpleMatrix currentVector = new SimpleMatrix(2,1);

    Double[] blueGoalPose = {16.0,131.0};
    Double[] initialRobotPose = new Double[2];
    Double initialHeading;
    Double currentHeading;
    double headingOffset;
    double imuAngle;

    double Kt = 2.57;
    double multiplier = 1;


    public void init() {
        robotHardware.init(hardwareMap);
        mecanumDrive.init(hardwareMap);

        robotHardware.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.turretMotor.setPositionPIDFCoefficients(17.5);


        fileContents = ReadWriteFile.readFile(file = AppUtil.getInstance().getSettingsFile("endPose.txt")).split(" ");
        initialRobotPose[0] = Double.parseDouble(fileContents[0]); initialRobotPose[1] = Double.parseDouble(fileContents[1]); initialHeading = Double.parseDouble(fileContents[2]);

        initialVector.set(0,0,blueGoalPose[0]-initialRobotPose[0]);
        initialVector.set(1,0,blueGoalPose[1]-initialRobotPose[1]);

        currentVector.set(0,0,blueGoalPose[0]-initialRobotPose[0]);
        currentVector.set(1,0,blueGoalPose[1]-initialRobotPose[1]);

        SparkFunOTOS.Pose2D initialPose = new SparkFunOTOS.Pose2D(initialRobotPose[0],initialRobotPose[1],initialHeading);
        robotHardware.sparkFunOTOS.setPosition(initialPose);

        robotHardware.limelight.setPollRateHz(100);
        robotHardware.limelight.pipelineSwitch(1);
    }

    public void start() {
        robotHardware.limelight.start();
    }

    public void loop() {
        LLResult llResult = robotHardware.limelight.getLatestResult();

        imuAngle = robotHardware.imu.getAngularOrientation().firstAngle;
        if (gamepad1.start) {
            headingOffset = -imuAngle;
        }
        double botHeading = -imuAngle - headingOffset;
        mecanumDrive.drive(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,botHeading);



        currentVector.set(0,0,blueGoalPose[0]-robotHardware.sparkFunOTOS.getPosition().x);
        currentVector.set(1,0,blueGoalPose[1]-robotHardware.sparkFunOTOS.getPosition().y);
        currentHeading = robotHardware.sparkFunOTOS.getPosition().h;

        double translationalAngle = Math.acos(initialVector.dot(currentVector) / (calculateMagnitude(initialVector.get(0,0),initialVector.get(1,0)) * calculateMagnitude(currentVector.get(0,0),currentVector.get(1,0))));

        if (robotHardware.sparkFunOTOS.getPosition().x - initialRobotPose[0] < 0) {
            multiplier = 1;
        } else {
            multiplier = -1;
        }

        double angle = multiplier * translationalAngle;
        double angleRotation = angle + robotHardware.imu.getAngularOrientation().firstAngle;

        robotHardware.turretMotor.setTargetPosition((int) (Kt * (Math.toDegrees(angleRotation) - llResult.getTx())));
        robotHardware.turretMotor.setPower(1);


        telemetry.addData("X: ", fileContents[0]);
        telemetry.addData("Y: ", fileContents[1]);
        telemetry.addData("H: ", fileContents[2]);
        telemetry.addData("XC: ", robotHardware.sparkFunOTOS.getPosition().x);
        telemetry.addData("YC: ", robotHardware.sparkFunOTOS.getPosition().y);
        telemetry.addData("HC: ", robotHardware.sparkFunOTOS.getPosition().h);
        telemetry.addData("Angle: ", Math.toDegrees(translationalAngle));
        telemetry.addData("Dot: ", initialVector.dot(currentVector));
        telemetry.addData("Vector Current: ", currentVector.get(0,0) + " " + currentVector.get(1,0));
        telemetry.addData("Vector Initial: ", initialVector.get(0,0) + " " + initialVector.get(1,0));



        telemetry.addData("TX: ", llResult.getTx());
    }

    public double calculateMagnitude(double x, double y) {
        return Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
    }



}
