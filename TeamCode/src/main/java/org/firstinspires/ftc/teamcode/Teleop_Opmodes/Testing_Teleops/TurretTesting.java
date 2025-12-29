package org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OLD_CLASSES.LimeLightTrackingAndDistance;
import org.firstinspires.ftc.teamcode.OLD_CLASSES.MecanumDriveConsentric;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;
import org.firstinspires.ftc.teamcode.OLD_CLASSES.TurretLQRController;


@Configurable
public class TurretTesting extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    MecanumDriveConsentric mecanumDriveConsentric;
    LimeLightTrackingAndDistance limeLightTrackingAndDistance;
    TurretLQRController lqrController;
    int turretPos;

    double previousAngle = 0;
    double headingOffset;



    double yPos;
    double xPos;
    double initialDistance;
    boolean started = false;

    double Kmt = 2.57;

    public void init() {
        SparkFunOTOS.Pose2D pose2D = new SparkFunOTOS.Pose2D(0,0,-90);


        robotHardware.init(hardwareMap);
        mecanumDriveConsentric = new MecanumDriveConsentric(robotHardware.frontLeftMotor, robotHardware.backLeftMotor, robotHardware.frontRightMotor, robotHardware.backRightMotor, robotHardware.imu);
        turretPos = robotHardware.turretMotor.getCurrentPosition();
        robotHardware.turretMotor.setTargetPositionTolerance(0);
        limeLightTrackingAndDistance = new LimeLightTrackingAndDistance(robotHardware.turretMotor);


        robotHardware.limelight.setPollRateHz(100);
        robotHardware.limelight.pipelineSwitch(0);


        headingOffset = robotHardware.imu.getAngularOrientation().firstAngle;
        robotHardware.turretMotor.setPositionPIDFCoefficients(17.5);

        robotHardware.sparkFunOTOS.setOffset(pose2D);
        robotHardware.sparkFunOTOS.calibrateImu();
        robotHardware.sparkFunOTOS.resetTracking();

        robotHardware.sparkFunOTOS.setAngularScalar(0.997);
        robotHardware.sparkFunOTOS.setLinearUnit(DistanceUnit.CM);
        robotHardware.sparkFunOTOS.setLinearScalar(1.006);

        lqrController = new TurretLQRController();

    }

    public void start() {
        robotHardware.limelight.start();
    }

    public void loop() {

        robotHardware.intakeServoRight.setPosition(0.34);
        robotHardware.intakeServoLeft.setPosition(0.66);



        double voltage = lqrController.updateLQR(5, robotHardware.turretMotor.getVelocity(AngleUnit.RADIANS));

        robotHardware.shooterMotorTop.setPower(voltage / 12);
        robotHardware.shooterMotorBottom.setPower(-voltage / 12);




        LLResult llResult = robotHardware.limelight.getLatestResult();
        limeLightTrackingAndDistance.setLLResult(llResult);

        if (!started) {
            initialDistance = limeLightTrackingAndDistance.distanceToTarget();
            started = true;
        }

        double limeLightError = llResult.getTx();

        yPos = robotHardware.sparkFunOTOS.getPosition().y;
        xPos = robotHardware.sparkFunOTOS.getPosition().x;

        double translationErrorCorrectionAngle = Math.toDegrees(Math.atan(xPos / (initialDistance - yPos)));
        double rotationError = robotHardware.imu.getAngularOrientation().firstAngle;
        double velocityFeedforwardAngle = robotHardware.imu.getAngularVelocity().xRotationRate;

        double trueAngleError = (rotationError - translationErrorCorrectionAngle + velocityFeedforwardAngle) + limeLightError;

        double motorOutpout = Kmt * trueAngleError;




        mecanumDriveConsentric.controlerDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,robotHardware.imu.getAngularOrientation().firstAngle,headingOffset);

        robotHardware.turretMotor.setTargetPosition((int) motorOutpout);
        robotHardware.turretMotor.setPower(1);



        telemetry.addData("pose X: ", robotHardware.sparkFunOTOS.getPosition().x);
        telemetry.addData("pose Y: ", robotHardware.sparkFunOTOS.getPosition().y);
        telemetry.addData("OTOS Heading: ", robotHardware.sparkFunOTOS.getPosition().h);

        telemetry.addData("","");

//        telemetry.addData("Current Heading", robotHardware.imu.getAngularOrientation().firstAngle);
//        telemetry.addData("Current TX: ", llResult.getTx());
//        telemetry.addData("limelight error: ", limeLightError);
//        telemetry.addData("Expected Turret Position: ", motorOutpout);
//        telemetry.addData("Current Turret Position: ", robotHardware.turretMotor.getCurrentPosition());
//        telemetry.addData("Rotation Error: ", rotationError);
//        telemetry.addData("Translation Error: ", translationErrorCorrectionAngle);
//        telemetry.addData("True Error: ", trueAngleError);

        telemetry.addData("Voltage: ", voltage);
        telemetry.addData("Current Motor voltage: ", robotHardware.shooterMotorTop.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Shooter Top Velocity R: ", robotHardware.shooterMotorTop.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Shooter Bottom Velocity R: ", robotHardware.shooterMotorBottom.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Shooter Top Velocity MT: ", robotHardware.shooterMotorTop.getVelocity());
        telemetry.addData("Shooter Bottom Velocity MT: ", robotHardware.shooterMotorBottom.getVelocity());

    }




}

