package org.firstinspires.ftc.teamcode.Teleop_Opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.IndexingClass;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.ShooterDistanceAndVelocity;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.TurretPositionPIDController;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.MecanumDrive;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.TeleopShootingAndIntaking;

@TeleOp(name = "Red Alliance", group = "Comp Teleop")
public class Red_Team_Main extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    TurretPositionPIDController turretPIDController = new TurretPositionPIDController();
    MecanumDrive mecanumDrive = new MecanumDrive();
    TeleopShootingAndIntaking shootingAndIntaking = new TeleopShootingAndIntaking();
    IndexingClass indexingClass = new IndexingClass();
    ShooterDistanceAndVelocity shooterDistanceAndVelocity = new ShooterDistanceAndVelocity();

    //------------------ Variables --------------------------------
    double initialDistance;
    double headingOffset;
    boolean started = false;

    //------------------ Constants --------------------------------
    double SERVO_INTAKE_POS_RIGHT = 0.34;
    double SERVO_INTAKE_POS_LEFT = 0.66;
    double SERVO_TRAVEL_POS_RIGHT = 0.5;
    double SERVO_TRAVEL_POS_LEFT = 0.5;
    int RED_TEAM_PIPELINE = 0;




    public void init() {
        //---------------- Class Initialization ----------------------
        robotHardware.init(hardwareMap);
        mecanumDrive.init(hardwareMap);
        shootingAndIntaking.init(hardwareMap);
        indexingClass.init(hardwareMap);

        //---------------Other Initialization --------------------------------
        robotHardware.limelight.setPollRateHz(100);
        robotHardware.limelight.pipelineSwitch(RED_TEAM_PIPELINE);

        SparkFunOTOS.Pose2D offsetPose = new SparkFunOTOS.Pose2D(0,0,-90);

    }



    public void start() {robotHardware.limelight.start();}




    public void loop() {
        double imuAngle = Math.toDegrees(robotHardware.imu.getAngularOrientation().firstAngle);
        LLResult llResult = robotHardware.limelight.getLatestResult();



//--------------------------------- Mecanum Driving --------------------------------
        if (gamepad1.start) {
            headingOffset = robotHardware.imu.getAngularOrientation().firstAngle;;
        }
        mecanumDrive.drive(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,headingOffset);


//---------------------------- Turret Positioning Code -----------------------------
        if (!started) {
            initialDistance = shooterDistanceAndVelocity.distanceToTarget(llResult.getTa());
            started = true;
        }

        double xPos = robotHardware.sparkFunOTOS.getPosition().x;
        double yPos = robotHardware.sparkFunOTOS.getPosition().y;
        double currentTXError = llResult.getTx();

        robotHardware.turretMotor.setPower(
                turretPIDController.updatePosition(
                        robotHardware.turretMotor.getCurrentPosition(),xPos,yPos,imuAngle,currentTXError,initialDistance));

        telemetry.addData("IMU Angle; ", imuAngle);
        telemetry.addData("TX: ", currentTXError);
        telemetry.addData("X Position: ", xPos);
        telemetry.addData("Y Position: ", yPos);


//--------------------------------- Indexing, Intaking, & Shooting ---------------------------------------
        boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;

        if (rightTriggerPressed) {
//            shootingAndIntaking.launchArtifacts();
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


//        robotHardware.shooterMotorTop.setPower(-0.75);
//        robotHardware.shooterMotorBottom.setPower(0.75);


        telemetry.addData("States: ", indexingClass.indexerStates);
        telemetry.addData("Pos A: ", indexingClass.doesPosAHaveArtifact());
        telemetry.addData("Pos B: ", indexingClass.doesPosBHaveArtifact());
        telemetry.addData("Pos C: ", indexingClass.doesPosCHaveArtifact());

    }

}
