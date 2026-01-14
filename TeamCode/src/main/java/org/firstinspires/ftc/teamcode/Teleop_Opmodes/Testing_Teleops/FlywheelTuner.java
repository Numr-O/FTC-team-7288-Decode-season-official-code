package org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.IndexingClass;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;


public class FlywheelTuner extends OpMode {
    RobotHardware robotHardware = new RobotHardware();

    double highVelocity = 330;
    double lowVelocity = 200;
    double currentVelocity = highVelocity;

    double SERVO_INTAKE_POS_RIGHT = 0.34;
    double SERVO_INTAKE_POS_LEFT = 0.66;
    double SERVO_TRANSFER_POS_RIGHT = 0.63;
    double SERVO_TRANSFER_POS_LEFT = 0.4;

    double P = 200;
    double F = 17.334;

    double[] stepSizes = {10.0,1.0,0.1,0.01,0.001,0.0001};

    int stepIndex = 1;

    public void init() {
        robotHardware.init(hardwareMap);
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


        robotHardware.shooterMotorTop.setVelocity(-currentVelocity, AngleUnit.DEGREES);
        robotHardware.shooterMotorBottom.setVelocity(currentVelocity, AngleUnit.DEGREES);

        double curVelocity = robotHardware.shooterMotorBottom.getVelocity(AngleUnit.DEGREES);
        double error = currentVelocity - curVelocity;

        telemetry.addData("current Velocity: ", curVelocity);
        telemetry.addData("target Velocity: ", currentVelocity);
        telemetry.addData("Error: ", error);
        telemetry.addData("----------------","-------------");
        telemetry.addData("Tuning F: ", F);
        telemetry.addData("Tuning P: ", P);
        telemetry.addData("Step Size: ", stepSizes[stepIndex]);

        if (gamepad1.right_trigger > 0.5) {
            robotHardware.indexerServo.setPosition(0.95);
            robotHardware.intakeMotor.setPower(1);
            robotHardware.intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
            robotHardware.intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);
        } else {
            robotHardware.intakeMotor.setPower(0);
            robotHardware.intakeServoRight.setPosition(SERVO_INTAKE_POS_RIGHT);
            robotHardware.intakeServoLeft.setPosition(SERVO_INTAKE_POS_LEFT);
        }


    }

}
