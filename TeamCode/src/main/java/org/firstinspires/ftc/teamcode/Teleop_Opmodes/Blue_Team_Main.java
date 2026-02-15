package org.firstinspires.ftc.teamcode.Teleop_Opmodes;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.IndexingClass;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.ShooterDistanceAndVelocity;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.TurretPositionUpdater;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.MecanumDrive;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.TeleopShootingAndIntaking;

import java.io.File;


@TeleOp(name = "Blue Alliance", group = "Comp Teleop")
public class Blue_Team_Main extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
//    TurretPositionPIDController turretPIDController = new TurretPositionPIDController();
    MecanumDrive mecanumDrive = new MecanumDrive();
    TeleopShootingAndIntaking shootingAndIntaking = new TeleopShootingAndIntaking();
    IndexingClass indexingClass = new IndexingClass();
    ShooterDistanceAndVelocity shooterDistanceAndVelocity = new ShooterDistanceAndVelocity();
    TurretPositionUpdater turretPositionUpdater = new TurretPositionUpdater();


    File file;
    ElapsedTime timer;


    //------------------ Variables --------------------------------
    double initialAngle;
    double headingOffset;
    double botHeading;
    boolean started = false;
    boolean grabValue = true;
    double shooterSpeed = 0;
    String[] fileContents;



    //------------------ Constants --------------------------------
    double SERVO_INTAKE_POS_RIGHT = 0.37;
    double SERVO_INTAKE_POS_LEFT = 0.63;
    double SERVO_TRAVEL_POS_RIGHT = 0.5;
    double SERVO_TRAVEL_POS_LEFT = 0.5;
    int RED_TEAM_PIPELINE = 1;

    Double[] initialRobotPose = new Double[2];
    Double initialHeading;
    Double robotStartingTicks;




    public void init() {
        //---------------- Class Initialization ----------------------
        fileContents = ReadWriteFile.readFile(file = AppUtil.getInstance().getSettingsFile("endPose.txt")).split(" ");
        initialRobotPose[0] = Double.parseDouble(fileContents[0]); initialRobotPose[1] = Double.parseDouble(fileContents[1]); initialHeading = Double.parseDouble(fileContents[2]);

        robotHardware.init(hardwareMap);
        mecanumDrive.init(hardwareMap);
        shootingAndIntaking.init(hardwareMap);
        indexingClass.init(hardwareMap);
        turretPositionUpdater.init(initialRobotPose, initialHeading,RED_TEAM_PIPELINE);

        timer = new ElapsedTime();



        //---------------Other Initialization --------------------------------



        robotHardware.limelight.setPollRateHz(100);
        robotHardware.limelight.pipelineSwitch(RED_TEAM_PIPELINE);

        SparkFunOTOS.Pose2D initialPose = new SparkFunOTOS.Pose2D(initialRobotPose[0],initialRobotPose[1],initialHeading);
        robotHardware.sparkFunOTOS.setPosition(initialPose);

        robotHardware.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.turretMotor.setPositionPIDFCoefficients(17.5);

        headingOffset = -robotHardware.imu.getAngularOrientation().firstAngle;

        telemetry.addData("Init ", "Done");

    }



    public void start() {
        robotHardware.limelight.start();
        robotHardware.turretMotor.setTargetPosition(0);
        robotHardware.turretMotor.setPower(0.5);
    }




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



        double xPos = robotHardware.sparkFunOTOS.getPosition().x;
        double yPos = robotHardware.sparkFunOTOS.getPosition().y;
        double currentHeading = robotHardware.sparkFunOTOS.getPosition().h;
        double currentTXError = llResult.getTx();





        telemetry.addData("IMU Angle; ", Math.toDegrees(imuAngle));
        telemetry.addData("TX: ", currentTXError);
        telemetry.addData("X Position: ", xPos);
        telemetry.addData("Y Position: ", yPos);
        telemetry.addData("Distance LL: ", shooterDistanceAndVelocity.distanceToTarget(llResult.getTa()));
        telemetry.addData("Distance OTOS: ", turretPositionUpdater.distanceToGoal());

        telemetry.addData("","");


//--------------------------------- Indexing, Intaking, & Shooting ---------------------------------------
        boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;




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


        telemetry.addData("Current Pos: ",robotHardware.turretMotor.getCurrentPosition());
        telemetry.addData("Target Pos: ",robotHardware.turretMotor.getTargetPosition());

        telemetry.addData("LL not times Kt: ", turretPositionUpdater.updatePosition(currentTXError,imuAngle,xPos,yPos));
        telemetry.addData("LL times Kt: ", turretPositionUpdater.updatePositionTwo(currentTXError,imuAngle,xPos,yPos));

        telemetry.addData("No LL: ", turretPositionUpdater.updatePositionOfTurret(imuAngle,xPos,yPos));

        telemetry.addData("","");

        if (rightTriggerPressed) {
            double offset = currentTXError * 2.57;
            int currentTurretPos = robotHardware.turretMotor.getCurrentPosition();

            int newTargetLL = currentTurretPos - (int) offset;
            int newTargetUpdater = turretPositionUpdater.updatePositionOfTurret(imuAngle,xPos,yPos);
//            robotHardware.turretMotor.setTargetPosition(newTargetLL);
            robotHardware.turretMotor.setTargetPosition((!llResult.isValid()) ? newTargetUpdater : newTargetLL);
//          robotHardware.turretMotor.setTargetPosition(turretPositionUpdater.updatePositionPositionOfTurret(currentTXError,imuAngle,xPos,yPos));
            robotHardware.turretMotor.setPower(1);


//            if (grabValue) {
//                if (llResult.isValid()) {
//                    shooterSpeed = shooterDistanceAndVelocity.distanceToSpeed(shooterDistanceAndVelocity.distanceToTarget(llResult.getTa()));
//                    grabValue = false;
//                } else {
//                    shooterSpeed = 900;
//                }
//            }
            double shooterSpeed = shooterDistanceAndVelocity.distanceToSpeed(turretPositionUpdater.distanceToGoal());
            setShooterVelocity(shooterSpeed);

            shootingAndIntaking.launchArtifactTwo(robotHardware.shooterMotorBottom.getVelocity() < shooterSpeed + 20 && robotHardware.shooterMotorBottom.getVelocity() > shooterSpeed - 20, indexingClass.doesPosBHaveArtifact());
            indexingClass.emptyIndexerArray();
        } else {
            grabValue = true;

            if (indexingClass.indexerStates == IndexingClass.IndexerStates.INDEXER_FULL) {
                robotHardware.ledLight.setPosition(0.5);
            } else {
                robotHardware.ledLight.setPosition(0.280);
            }

            indexingClass.indexArtifacts();
            setShooterSpeedZero();
        }




        telemetry.addData("States: ", indexingClass.indexerStates);
        telemetry.addData("Array: ", indexingClass.indexerPositions[0] + " " + indexingClass.indexerPositions[1] + " " + indexingClass.indexerPositions[2]);
        telemetry.addData("Pos A: ", indexingClass.doesPosAHaveArtifact());
        telemetry.addData("Pos B: ", indexingClass.doesPosBHaveArtifact());
        telemetry.addData("Pos C: ", indexingClass.doesPosCHaveArtifact());
    }




    public void setShooterVelocity (double vel) {
        robotHardware.shooterMotorBottom.setVelocity(vel);
        robotHardware.shooterMotorTop.setVelocity(-vel);

    }
    public void setShooterSpeedZero() {
        robotHardware.shooterMotorBottom.setPower(0);
        robotHardware.shooterMotorTop.setPower(0);
    }

}
