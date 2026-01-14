package org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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


    double velocity = 0;

    public void init() {
        robotHardware.init(hardwareMap);
        indexingClass.init(hardwareMap);
        shootingAndIntaking.init(hardwareMap);


        robotHardware.limelight.setPollRateHz(100);
        robotHardware.limelight.pipelineSwitch(0);
    }

    public void start() {robotHardware.limelight.start();}

    public void loop() {

        boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;

        if (rightTriggerPressed) {
            shootingAndIntaking.launchArtifacts();
            indexingClass.emptyIndexerArray();
        } else {
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

        if (gamepad1.aWasReleased() && velocity < 1000) {
            velocity += 5;
        } else if (gamepad1.bWasReleased() && velocity > 0) {
            velocity -= 5;
        }

        robotHardware.shooterMotorBottom.setVelocity(velocity, AngleUnit.DEGREES);
        robotHardware.shooterMotorTop.setVelocity(-velocity, AngleUnit.DEGREES);


        LLResult llResult = robotHardware.limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        for(LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == 24) {
                telemetry.addData("Distance POSE: ", fiducial.getCameraPoseTargetSpace().getPosition().z);
            }
        }

        telemetry.addData("VELOCITY: ", velocity);
        telemetry.addData("DISTANCE: ", shooterDistanceVelocity.distanceToTarget(llResult.getTa()));
        telemetry.addData("Distance OTOS X: ", robotHardware.sparkFunOTOS.getPosition().x);
        telemetry.addData("Distance OTOS Y: ", robotHardware.sparkFunOTOS.getPosition().y);
        telemetry.addData("angle: ", robotHardware.sparkFunOTOS.getPosition().h);


    }

}
