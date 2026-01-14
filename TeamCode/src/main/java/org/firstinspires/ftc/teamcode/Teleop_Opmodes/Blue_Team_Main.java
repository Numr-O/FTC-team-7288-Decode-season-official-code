package org.firstinspires.ftc.teamcode.Teleop_Opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.IndexingClass;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.ShooterDistanceAndVelocity;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.TurretPositionPIDController;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.MecanumDrive;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.TeleopShootingAndIntaking;

@TeleOp(name = "Blue Alliance", group = "Comp Teleop")
public class Blue_Team_Main extends OpMode {
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

//    LUT<Double,Double> shooterVelocities = new LUT<Double,Double>()
//    {{
//        shooterVelocities.add(1.0,1.0);
//
//    }};

    //------------------ Constants --------------------------------
    double SERVO_INTAKE_POS_RIGHT = 0.34;
    double SERVO_INTAKE_POS_LEFT = 0.66;
    double SERVO_TRAVEL_POS_RIGHT = 0.5;
    double SERVO_TRAVEL_POS_LEFT = 0.5;
    int BLUE_TEAM_PIPELINE = 1;

    int[] SHOOTER_VELOCITIES_DEGREES = {348,300,275,4};


    public void init() {
        //---------------- Class Initialization ----------------------
        robotHardware.init(hardwareMap);
        mecanumDrive.init(hardwareMap);
        shootingAndIntaking.init(hardwareMap);
        indexingClass.init(hardwareMap);

        //---------------Other Initialization --------------------------------
        robotHardware.limelight.setPollRateHz(100);
        robotHardware.limelight.pipelineSwitch(BLUE_TEAM_PIPELINE);

        SparkFunOTOS.Pose2D offsetPose = new SparkFunOTOS.Pose2D(0,0,-90);
        robotHardware.sparkFunOTOS.setPosition(offsetPose);

        robotHardware.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.turretMotor.setPositionPIDFCoefficients(15);

        telemetry.addData("Init ", "Done");

    }



    public void start() {robotHardware.limelight.start();}




    public void loop() {
        double imuAngle = robotHardware.imu.getAngularOrientation().firstAngle;
        LLResult llResult = robotHardware.limelight.getLatestResult();



//--------------------------------- Mecanum Driving --------------------------------
        if (gamepad1.start) {
            headingOffset = robotHardware.imu.getAngularOrientation().firstAngle;
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

        robotHardware.turretMotor.setTargetPosition(
                turretPIDController.updatePosition(
                        xPos,yPos,Math.toDegrees(imuAngle),currentTXError,initialDistance));
        robotHardware.turretMotor.setPower(1);

        telemetry.addData("IMU Angle; ", Math.toDegrees(imuAngle));
        telemetry.addData("TX: ", currentTXError);
        telemetry.addData("X Position: ", xPos);
        telemetry.addData("Y Position: ", yPos);
        telemetry.addData("Distance: ", shooterDistanceAndVelocity.distanceToTarget(llResult.getTa()));
        telemetry.addData("Current Pos: ",robotHardware.turretMotor.getCurrentPosition());


//--------------------------------- Indexing, Intaking, & Shooting ---------------------------------------
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

        double distanceCM = shooterDistanceAndVelocity.distanceToTarget(llResult.getTa());
        if (distanceCM > 300) {
            setShooterVelocity(SHOOTER_VELOCITIES_DEGREES[0]);
        } else if (distanceCM < 200 && distanceCM > 160) {
            setShooterVelocity(SHOOTER_VELOCITIES_DEGREES[1]);
        } else if (distanceCM < 150 && distanceCM > 100) {
            setShooterVelocity(SHOOTER_VELOCITIES_DEGREES[2]);
        }




        telemetry.addData("States: ", indexingClass.indexerStates);
        telemetry.addData("Array: ", indexingClass.indexerPositions[0] + " " + indexingClass.indexerPositions[1] + " " + indexingClass.indexerPositions[2]);
        telemetry.addData("Pos A: ", indexingClass.doesPosAHaveArtifact());
        telemetry.addData("Pos B: ", indexingClass.doesPosBHaveArtifact());
        telemetry.addData("Pos C: ", indexingClass.doesPosCHaveArtifact());

    }

    public void setShooterVelocity (double vel) {
        robotHardware.shooterMotorBottom.setVelocity(vel, AngleUnit.DEGREES);
        robotHardware.shooterMotorTop.setVelocity(-vel, AngleUnit.DEGREES);

    }

}
