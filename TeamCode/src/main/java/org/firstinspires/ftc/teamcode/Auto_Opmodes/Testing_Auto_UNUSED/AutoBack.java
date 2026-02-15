package org.firstinspires.ftc.teamcode.Auto_Opmodes.Testing_Auto_UNUSED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OLD_CLASSES.IndexerShootingAndIntake;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;

public class AutoBack extends OpMode {

    RobotHardware robothwde = new RobotHardware();
    ElapsedTime timer;
    IndexerShootingAndIntake indexerShootingAndIntake;
    Boolean done = false;
    @Override
    public void init() {
        robothwde.init(hardwareMap);

        Object[] hardwareList = {robothwde.indexerServo,robothwde.intakeServoLeft,robothwde.intakeServoRight,robothwde.shooterMotorTop,robothwde.shooterMotorBottom,robothwde.intakeMotor,robothwde.colorPosA,robothwde.colorPosB,robothwde.colorPosC,robothwde.ledLight};
        indexerShootingAndIntake = new IndexerShootingAndIntake(hardwareList);


        timer = new ElapsedTime();
    }

    @Override
    public void start() {

    }

    @Override
    public void  loop() {

        indexerShootingAndIntake.autoShooting(1450);

        if (indexerShootingAndIntake.test == 4 && !done) {


            timer.reset();
            while (timer.milliseconds() < 1500) {
                robothwde.frontLeftMotor.setPower(0.3);
                robothwde.backLeftMotor.setPower(0.3);
                robothwde.frontRightMotor.setPower(0.3);
                robothwde.backRightMotor.setPower(0.3);
            }
            robothwde.frontLeftMotor.setPower(0);
            robothwde.backLeftMotor.setPower(0);
            robothwde.frontRightMotor.setPower(0);
            robothwde.backRightMotor.setPower(0);
            robothwde.shooterMotorBottom.setVelocity(0);
            robothwde.shooterMotorBottom.setVelocity(0);
            done = true;
        }
    }

}
