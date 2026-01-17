package org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.IndexingClass;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;


public class TeleopShootingAndIntaking extends IndexingClass{
    RobotHardware robotHardware = new RobotHardware();
    IndexingClass indexingClass = new IndexingClass();
    ElapsedTime timer;

    Servo intakeServoLeft;
    Servo intakeServoRight;
    Servo indexerServo;

    DcMotor intakeMotor;

    //------------------------Constants---------------------------------------

    double INDEXER_SERVO_POS_A_EXTRA = 0.97;

    double SERVO_TRANSFER_POS_RIGHT = 0.63;
    double SERVO_TRANSFER_POS_LEFT = 0.4;




    //--------------------------------Initializing---------------------------
    public void init(HardwareMap hardwareMap) {
        robotHardware.init(hardwareMap);
        timer = new ElapsedTime();


//        this.intakeServoLeft = hardwareMap.get(Servo.class, "intakeL");
//        this.intakeServoRight = hardwareMap.get(Servo.class, "intakeR");
//
//        this.indexerServo = hardwareMap.get(Servo.class, "indexer");
//
//        intakeMotor = hardwareMap.get(DcMotor.class, "test");
//        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
//        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    //----------------------------------Class Functions-------------------------------
//    public void indexingOrShooting(boolean isTriggerPressed) {
//        if (isTriggerPressed) {
//            intakeMotor.setPower(1);
//            intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
//            intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);
//            timer.reset();
//            if (timer.milliseconds()>100) {indexerServo.setPosition(INDEXER_SERVO_POS_A_EXTRA);}
//
//        } else {
//            indexingClass.indexArtifacts();
//        }
//    }

    public void launchArtifacts() {
        robotHardware.intakeMotor.setPower(1);
        robotHardware.intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
        robotHardware.intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);
        timer.reset();
        while (timer.milliseconds()<100);
        robotHardware.indexerServo.setPosition(INDEXER_SERVO_POS_A_EXTRA);
    }

    public void launchArtifactTwo(boolean isShooterAtSpeed) {
        if (isShooterAtSpeed) {
            robotHardware.intakeMotor.setPower(1);
            robotHardware.intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
            robotHardware.intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);
            timer.reset();
            while (timer.milliseconds() < 100) ;
            robotHardware.indexerServo.setPosition(INDEXER_SERVO_POS_A_EXTRA);
        }
    }




}
