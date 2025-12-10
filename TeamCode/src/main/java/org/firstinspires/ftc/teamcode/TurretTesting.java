package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.mainCode.MecanumDriveConsentric;
import org.firstinspires.ftc.teamcode.mainCode.RobotHardware;

@TeleOp
public class TurretTesting extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    MecanumDriveConsentric mecanumDriveConsentric;
    int turretPos;

    double previousAngle = 0;
    double headingOffset;

    public void init() {
        robotHardware.init(hardwareMap);
        mecanumDriveConsentric = new MecanumDriveConsentric(robotHardware.frontLeftMotor, robotHardware.backLeftMotor, robotHardware.frontRightMotor, robotHardware.backRightMotor, robotHardware.imu);
        turretPos = robotHardware.turretMotor.getCurrentPosition();
        robotHardware.turretMotor.setTargetPositionTolerance(0);
        robotHardware.limelight.setPollRateHz(100);
        robotHardware.limelight.pipelineSwitch(0);
        robotHardware.revIMU.init();
        headingOffset = robotHardware.imu.getAngularOrientation().firstAngle;
        robotHardware.turretMotor.setPositionPIDFCoefficients(12);


    }

    public void start() {
        robotHardware.limelight.start();
    }

    public void loop() {
        mecanumDriveConsentric.controlerDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,headingOffset);

        LLResult llResult = robotHardware.limelight.getLatestResult();
        if (gamepad1.aWasReleased()) {
            turretPos += 1;
        } else if (gamepad1.bWasReleased()) {
            turretPos -= 1;
        }
        if (gamepad1.start) {

        }
        int currentPos = robotHardware.turretMotor.getCurrentPosition();
        int limeLightError = (int) Math.floor(llResult.getTx());
        int angleError = (int) (2.5 * robotHardware.revIMU.getAbsoluteHeading());
        int targetPos = (!llResult.isValid()) ? angleError : currentPos + limeLightError;
        int targetPosTest = angleError - limeLightError;

        robotHardware.turretMotor.setTargetPosition(targetPos);
        robotHardware.turretMotor.setPower(1);



        telemetry.addData("Current Heading", robotHardware.revIMU.getAbsoluteHeading());
        telemetry.addData("Current TX: ", llResult.getTx());
        telemetry.addData("limelight error: ", limeLightError);
        telemetry.addData("Expected Turret Position: ", targetPos);
        telemetry.addData("Current Turret Position: ", currentPos);
        telemetry.addData("P Term: ", robotHardware.turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("Acceleration: ", robotHardware.imu.getLinearAcceleration());


    }



}

