package org.firstinspires.ftc.teamcode.Teleop_Opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.IndexingClass;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.ShooterDistanceAndVelocity;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.TurretPositionPIDController;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.TurretPositionUpdater;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.MecanumDrive;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.TeleopShootingAndIntaking;

import java.io.File;

@TeleOp(name = "Red Alliance", group = "Comp Teleop")
public class Red_Team_Main extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
//    TurretPositionPIDController turretPIDController = new TurretPositionPIDController();
    MecanumDrive mecanumDrive = new MecanumDrive();
    TeleopShootingAndIntaking shootingAndIntaking = new TeleopShootingAndIntaking();
    IndexingClass indexingClass = new IndexingClass();
    ShooterDistanceAndVelocity shooterDistanceAndVelocity = new ShooterDistanceAndVelocity();
    TurretPositionUpdater turretPositionUpdater = new TurretPositionUpdater();
    File file;


    //------------------ Variables --------------------------------
    double initialDistance;
    double initialAngle;
    double headingOffset;
    double botHeading;
    boolean started = false;
    int shooterSpeed = 0;
    String[] fileContents;


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
    int RED_TEAM_PIPELINE = 0;

    int[] SHOOTER_VELOCITIES_DEGREES = {346,275,225,4};
    Double[] initialRobotPose = new Double[2];
    Double initialHeading;




    public void init() {
        //---------------- Class Initialization ----------------------
        fileContents = ReadWriteFile.readFile(file = AppUtil.getInstance().getSettingsFile("endPose.txt")).split(" ");
        initialRobotPose[0] = Double.parseDouble(fileContents[0]); initialRobotPose[1] = Double.parseDouble(fileContents[1]); initialHeading = Double.parseDouble(fileContents[2]);

        robotHardware.init(hardwareMap);
        mecanumDrive.init(hardwareMap);
        shootingAndIntaking.init(hardwareMap);
        indexingClass.init(hardwareMap);
        turretPositionUpdater.init(initialRobotPose, initialHeading,RED_TEAM_PIPELINE);



        //---------------Other Initialization --------------------------------


        robotHardware.limelight.setPollRateHz(100);
        robotHardware.limelight.pipelineSwitch(RED_TEAM_PIPELINE);

        SparkFunOTOS.Pose2D initialPose = new SparkFunOTOS.Pose2D(initialRobotPose[0],initialRobotPose[1],initialHeading);
        robotHardware.sparkFunOTOS.setPosition(initialPose);

        robotHardware.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.turretMotor.setPositionPIDFCoefficients(17.5);

        headingOffset = -robotHardware.imu.getAngularOrientation().firstAngle;

        telemetry.addData("Init ", "Done");

    }



    public void start() {robotHardware.limelight.start();}




    public void loop() {
        double imuAngle = robotHardware.imu.getAngularOrientation().firstAngle;
        LLResult llResult = robotHardware.limelight.getLatestResult();



//--------------------------------- Mecanum Driving --------------------------------
        if (gamepad1.start) {
            headingOffset = -imuAngle;
        }

        botHeading = -imuAngle - headingOffset;
        mecanumDrive.drive(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,botHeading);


//---------------------------- Turret Positioning Code -----------------------------
        if (!started) {
            initialDistance = shooterDistanceAndVelocity.distanceToTarget(llResult.getTa());
            started = true;
        }


        double xPos = robotHardware.sparkFunOTOS.getPosition().x;
        double yPos = robotHardware.sparkFunOTOS.getPosition().y;
        double currentHeading = robotHardware.sparkFunOTOS.getPosition().h;
        double currentTXError = llResult.getTx();

//        robotHardware.turretMotor.setTargetPosition(
//                turretPIDController.updatePosition(
//                        xPos,yPos,Math.toDegrees(imuAngle),currentTXError,initialDistance));
//        robotHardware.turretMotor.setPower(1);




        telemetry.addData("IMU Angle; ", Math.toDegrees(imuAngle));
        telemetry.addData("TX: ", currentTXError);
        telemetry.addData("X Position: ", xPos);
        telemetry.addData("Y Position: ", yPos);
        telemetry.addData("Distance: ", shooterDistanceAndVelocity.distanceToTarget(llResult.getTa()));
        telemetry.addData("Current Pos: ",robotHardware.turretMotor.getCurrentPosition());


//--------------------------------- Indexing, Intaking, & Shooting ---------------------------------------
        boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;

//        if (rightTriggerPressed) {
//            shootingAndIntaking.launchArtifacts();
//            indexingClass.emptyIndexerArray();
//        } else {
//            indexingClass.indexArtifacts();
//        }


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
        if (gamepad1.right_trigger > 0.5) {
            if (distanceCM > 300) {
              setShooterVelocity(SHOOTER_VELOCITIES_DEGREES[0]);
              shooterSpeed = SHOOTER_VELOCITIES_DEGREES[0];

            } else if (distanceCM < 250 && distanceCM > 160) {
              setShooterVelocity(SHOOTER_VELOCITIES_DEGREES[1]);
                shooterSpeed = SHOOTER_VELOCITIES_DEGREES[1];

            } else if (distanceCM < 159 && distanceCM > 100) {
                setShooterVelocity(SHOOTER_VELOCITIES_DEGREES[2]);
                shooterSpeed = SHOOTER_VELOCITIES_DEGREES[2];

            }
        } else {
            setShooterSpeedZero();
        }

        if (rightTriggerPressed) {
            robotHardware.turretMotor.setTargetPosition(turretPositionUpdater.updatePosition(currentTXError,imuAngle,xPos,yPos));
            robotHardware.turretMotor.setPower(1);

            shootingAndIntaking.launchArtifactTwo(robotHardware.shooterMotorBottom.getVelocity(AngleUnit.DEGREES) < shooterSpeed + 3 && robotHardware.shooterMotorBottom.getVelocity(AngleUnit.DEGREES) > shooterSpeed - 3);
            indexingClass.emptyIndexerArray();
            indexingClass.indexerStates = IndexingClass.IndexerStates.INDEX_TO_A;
        } else {
            indexingClass.indexArtifacts();
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
    public void setShooterSpeedZero() {
        robotHardware.shooterMotorBottom.setPower(0);
        robotHardware.shooterMotorTop.setPower(0);
    }

}
