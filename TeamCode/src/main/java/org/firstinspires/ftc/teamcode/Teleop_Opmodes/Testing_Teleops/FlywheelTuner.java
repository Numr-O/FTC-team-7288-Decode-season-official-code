package org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.IndexingClass;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.TeleopShootingAndIntaking;

import java.util.Timer;

@Configurable
public class FlywheelTuner extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    IndexingClass indexingClass = new IndexingClass();
    TeleopShootingAndIntaking shootingAndIntaking = new TeleopShootingAndIntaking();
    ElapsedTime timer;

    double highVelocity = 1400;
    double lowVelocity = 900;
    double currentVelocity = highVelocity;

    double SERVO_INTAKE_POS_RIGHT = 0.34;
    double SERVO_INTAKE_POS_LEFT = 0.66;
    double SERVO_TRANSFER_POS_RIGHT = 0.63;
    double SERVO_TRANSFER_POS_LEFT = 0.4;
    double INDEXER_SERVO_POS_A_EXTRA = 0.97;


    double P = 500;
    double F = 17.517;

    double[] stepSizes = {10.0,1.0,0.1,0.01,0.001,0.0001};

    int stepIndex = 1;

    public void init() {
        robotHardware.init(hardwareMap);
        indexingClass.init(hardwareMap);
        shootingAndIntaking.init(hardwareMap);
        timer = new ElapsedTime();
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        robotHardware.shooterMotorTop.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        robotHardware.shooterMotorBottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);


    }

    public void loop() {
        if (gamepad1.yWasPressed()) {
            if (currentVelocity == highVelocity) {
                currentVelocity = lowVelocity;
            } else {
                currentVelocity = highVelocity;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        robotHardware.shooterMotorTop.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        robotHardware.shooterMotorBottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        if (gamepad1.xWasPressed()) {
            if (robotHardware.shooterMotorBottom.getVelocity() < currentVelocity - 300 || robotHardware.shooterMotorBottom.getVelocity() > currentVelocity + 20) {
                robotHardware.shooterMotorTop.setVelocity(-currentVelocity);
                robotHardware.shooterMotorBottom.setVelocity(currentVelocity);
            } else {
                robotHardware.shooterMotorTop.setVelocity(0);
                robotHardware.shooterMotorBottom.setVelocity(0);
            }
        }


        double curVelocity = robotHardware.shooterMotorBottom.getVelocity();
        double error = currentVelocity - curVelocity;

        telemetry.addData("current Velocity: ", curVelocity);
        telemetry.addData("target Velocity: ", currentVelocity);
        telemetry.addData("Error: ", error);
        telemetry.addData("----------------","-------------");
        telemetry.addData("Tuning F: ", F);
        telemetry.addData("Tuning P: ", P);
        telemetry.addData("Step Size: ", stepSizes[stepIndex]);

        if (gamepad1.right_trigger > 0.5) {
            robotHardware.intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
            robotHardware.intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);
            robotHardware.intakeMotor.setPower(1);

            timer.reset();
            while(timer.milliseconds() < 500);

            robotHardware.indexerServo.setPosition(INDEXER_SERVO_POS_A_EXTRA);

            indexingClass.emptyIndexerArray();
            indexingClass.indexerStates = IndexingClass.IndexerStates.INDEX_TO_A;

        } else {
            if (gamepad1.left_trigger > 0.5) {
                robotHardware.intakeMotor.setPower(1);
            } else {
                robotHardware.intakeMotor.setPower(0);
            }

            robotHardware.intakeServoRight.setPosition(SERVO_INTAKE_POS_RIGHT);
            robotHardware.intakeServoLeft.setPosition(SERVO_INTAKE_POS_LEFT);

            indexingClass.indexArtifacts();
        }


    }

}
