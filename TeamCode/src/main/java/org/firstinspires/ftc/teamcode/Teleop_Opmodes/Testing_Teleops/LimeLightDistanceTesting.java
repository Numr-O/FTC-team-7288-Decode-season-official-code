package org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops;


import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.IndexingClass;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.ShooterDistanceAndVelocity;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.TeleopShootingAndIntaking;

import java.util.List;

@Configurable
public class LimeLightDistanceTesting extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    ShooterDistanceAndVelocity shooterDistanceVelocity = new ShooterDistanceAndVelocity();
    IndexingClass indexingClass = new IndexingClass();
    TeleopShootingAndIntaking shootingAndIntaking = new TeleopShootingAndIntaking();

    double SERVO_INTAKE_POS_RIGHT = 0.34;
    double SERVO_INTAKE_POS_LEFT = 0.66;
    double SERVO_TRANSFER_POS_RIGHT = 0.63;
    double SERVO_TRANSFER_POS_LEFT = 0.4;

    double SERVO_TRAVEL_POS_RIGHT = 0.5;
    double SERVO_TRAVEL_POS_LEFT = 0.5;

    double shooterSpeed = 0;
    double velocity = 0;
    boolean grabValue = true;

    public void init() {

        robotHardware.init(hardwareMap);
        indexingClass.init(hardwareMap);
        shootingAndIntaking.init(hardwareMap);


        robotHardware.limelight.setPollRateHz(100);
        robotHardware.limelight.pipelineSwitch(0);


    }

    public void start() {robotHardware.limelight.start();}

    public void loop() {

        LLResult llResult = robotHardware.limelight.getLatestResult();

        boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;


        if (rightTriggerPressed) {
            if (grabValue) {
                shooterSpeed = distanceToSpeed(shooterDistanceVelocity.distanceToTarget(llResult.getTa()));
                grabValue = false;
            }

            robotHardware.shooterMotorBottom.setVelocity(shooterSpeed);
            robotHardware.shooterMotorTop.setVelocity(-shooterSpeed);

            shootingAndIntaking.launchArtifactTwo(robotHardware.shooterMotorBottom.getVelocity() < shooterSpeed + 20 && robotHardware.shooterMotorBottom.getVelocity() > shooterSpeed - 20, indexingClass.doesPosBHaveArtifact());
            indexingClass.emptyIndexerArray();
            indexingClass.indexerStates = IndexingClass.IndexerStates.INDEX_TO_A;

        } else {
            grabValue = true;

            robotHardware.shooterMotorBottom.setVelocity(0);
            robotHardware.shooterMotorTop.setVelocity(0);

            indexingClass.indexArtifacts();
        }


        if (leftTriggerPressed && !rightTriggerPressed) {
            robotHardware.intakeServoRight.setPosition(SERVO_INTAKE_POS_RIGHT);
            robotHardware.intakeServoLeft.setPosition(SERVO_INTAKE_POS_LEFT);
            robotHardware.intakeMotor.setPower(1);
        } else if (gamepad1.left_bumper && !leftTriggerPressed && !rightTriggerPressed) {
            robotHardware.intakeServoRight.setPosition(SERVO_INTAKE_POS_RIGHT);
            robotHardware.intakeServoLeft.setPosition(SERVO_INTAKE_POS_LEFT);
            robotHardware.intakeMotor.setPower(-1);
        } else if (!rightTriggerPressed) {
            robotHardware.intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
            robotHardware.intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);
            robotHardware.intakeMotor.setPower(0);
        }

        if (gamepad1.aWasReleased() && velocity < 2000) {
            velocity += 20;
        } else if (gamepad1.bWasReleased() && velocity > 0) {
            velocity -= 20;
        }



        telemetry.addData("Set VELOCITY: ", velocity);
        telemetry.addData("Current VELOCITY: ", robotHardware.shooterMotorBottom.getVelocity());
        telemetry.addData("DISTANCE: ", shooterDistanceVelocity.distanceToTarget(llResult.getTa()));
        telemetry.addData("Distance OTOS X: ", robotHardware.sparkFunOTOS.getPosition().x);
        telemetry.addData("Distance OTOS Y: ", robotHardware.sparkFunOTOS.getPosition().y);
        telemetry.addData("angle: ", robotHardware.sparkFunOTOS.getPosition().h);


    }

   public double distanceToSpeed(double distance) {
        return 1.67279 * distance + 889.73139;
   }

}
