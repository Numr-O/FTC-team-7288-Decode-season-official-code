package org.firstinspires.ftc.teamcode.Auto_Opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OLD_CLASSES.IndexerShootingAndIntake;
import org.firstinspires.ftc.teamcode.OLD_CLASSES.LimeLightTrackingAndDistance;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;

@Autonomous
public class Auto extends OpMode {
    RobotHardware robothwde = new RobotHardware();
    LimeLightTrackingAndDistance limeLightTrackingAndDistance;
    IndexerShootingAndIntake indexerShootingAndIntake;
    ElapsedTime timer;

    boolean done;

    double numberOne;
    double numberTwo;

    String outputTele;

    @Override
    public void init() {
        robothwde.init(hardwareMap);
        limeLightTrackingAndDistance = new LimeLightTrackingAndDistance(robothwde.turretMotor);
        Object[] hardwareList = {robothwde.indexerServo,robothwde.intakeServoLeft,robothwde.intakeServoRight,robothwde.shooterMotorTop,robothwde.shooterMotorBottom,robothwde.intakeMotor,robothwde.colorPosA,robothwde.colorPosB,robothwde.colorPosC,robothwde.ledLight};
        indexerShootingAndIntake = new IndexerShootingAndIntake(hardwareList);
        timer = new ElapsedTime();
        robothwde.limelight.setPollRateHz(100);
        robothwde.ledLight.setPosition(1);
    }

    @Override
    public void init_loop() {

        if (gamepad1.aWasReleased()) {
            robothwde.limelight.pipelineSwitch(0);
            numberTwo = -1;
            numberOne = 1;
            outputTele = "RED";
            robothwde.ledLight.setPosition(0.280);
        } else if (gamepad1.bWasReleased()) {
            robothwde.limelight.pipelineSwitch(1);
            numberTwo = 1;
            numberOne = -1;
            outputTele = "BLUE";
            robothwde.ledLight.setPosition(0.666);
        }

        telemetry.addData("TEAM SELECT: ", outputTele);
    }

    @Override
    public void start() {
        robothwde.limelight.start();

        timer.reset();
        while (timer.milliseconds() < 1300) {
            robothwde.frontLeftMotor.setPower(-0.6);
            robothwde.backLeftMotor.setPower(-0.6);
            robothwde.frontRightMotor.setPower(-0.6);
            robothwde.backRightMotor.setPower(-0.6);
        }
        robothwde.frontLeftMotor.setPower(0);
        robothwde.backLeftMotor.setPower(0);
        robothwde.frontRightMotor.setPower(0);
        robothwde.backRightMotor.setPower(0);


    }

    @Override
    public void loop() {
        LLResult llResult = robothwde.limelight.getLatestResult();
        limeLightTrackingAndDistance.setLLResult(llResult);

        telemetry.addData("List: ", indexerShootingAndIntake.test);
        telemetry.addData("Is in State: ", indexerShootingAndIntake.isInState);
        telemetry.addData("Is Shooter At Speed:", indexerShootingAndIntake.areShooterMotorsAtSpeed);

        telemetry.addData("Shooter Velocity Top: ", robothwde.shooterMotorTop.getVelocity());
        telemetry.addData("Shooter Velocity Top: ", robothwde.shooterMotorTop.getVelocity());
        telemetry.addData("Shooter Velocity Expected: ", limeLightTrackingAndDistance.calculateRPMForShooter());

        indexerShootingAndIntake.autoShooting(limeLightTrackingAndDistance.calculateRPMForShooter());

        telemetry.addData("State Index: ", indexerShootingAndIntake.getIndexingState());

        if (indexerShootingAndIntake.test == 4 && !done) {
            robothwde.shooterMotorBottom.setVelocity(0);
            robothwde.shooterMotorTop.setVelocity(0);
            robothwde.intakeMotor.setPower(0);

            robothwde.frontLeftMotor.setPower(numberOne * 0.6);
            robothwde.backLeftMotor.setPower(numberTwo * 0.6);
            robothwde.frontRightMotor.setPower(numberTwo * 0.6);
            robothwde.backRightMotor.setPower(numberOne * 0.6);
            timer.reset();
            while (timer.milliseconds() < 1000);
            robothwde.frontLeftMotor.setPower(0);
            robothwde.backLeftMotor.setPower(0);
            robothwde.frontRightMotor.setPower(0);
            robothwde.backRightMotor.setPower(0);

            done = true;

        }


    }
}
