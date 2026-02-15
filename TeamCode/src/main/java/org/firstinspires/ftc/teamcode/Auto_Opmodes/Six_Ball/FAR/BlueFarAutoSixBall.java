package org.firstinspires.ftc.teamcode.Auto_Opmodes.Six_Ball.FAR;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.IndexingClass;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.TurretPositionUpdater;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.TeleopShootingAndIntaking;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.File;


public class BlueFarAutoSixBall extends OpMode {
    File file = AppUtil.getInstance().getSettingsFile("endPose.txt");
    RobotHardware robotHardware = new RobotHardware();
    IndexingClass indexingClass = new IndexingClass();
    TeleopShootingAndIntaking shootingAndIntaking = new TeleopShootingAndIntaking();
    TurretPositionUpdater turretPositionUpdater = new TurretPositionUpdater();
    Follower follower;
    Timer pathTimer;
    Timer timer;
    LLResult llResult;
    private int pathState;
    double pickupSpeed = 0.32 ;
    boolean startIndexing = true;
    int loops = 0;

    Double[] blueGoalPose = {16.0,131.0};
    Double[] redGoalPose = {128.0,131.0};


    boolean zero = false;

    double INDEXER_SERVO_POS_A_EXTRA = 0.97;

    double SERVO_INTAKE_POS_RIGHT = 0.37;
    double SERVO_INTAKE_POS_LEFT = 0.63;
    double SERVO_TRAVEL_POS_RIGHT = 0.5;
    double SERVO_TRAVEL_POS_LEFT = 0.5;
    double SERVO_TRANSFER_POS_RIGHT = 0.63;
    double SERVO_TRANSFER_POS_LEFT = 0.4;


    private final Pose startPose = new Pose(53.7,8,Math.toRadians(180));
    private final Pose shootPose = new Pose(50,8,Math.toRadians(180));
    private final Pose pickupPoseOne = new Pose(8,8,Math.toRadians(180));
    private final Pose shakePose = new Pose(8,12,Math.toRadians(160));
    private final Pose moveOffPose = new Pose(52.4,24.5, Math.toRadians(115));





    private PathChain startToShoot, shootToPickup, shootArtifacts, shakeFront ,shakeBack, moveOffPath;

    public void buildPaths() {

        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootToPickup = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPoseOne))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPoseOne.getHeading())
                .build();
        shootArtifacts = follower.pathBuilder()
                .addPath(new BezierLine(pickupPoseOne, shootPose))
                .setLinearHeadingInterpolation(pickupPoseOne.getHeading(), shootPose.getHeading())
                .build();
        shakeBack = follower.pathBuilder()
                .addPath(new BezierLine(shakePose, pickupPoseOne))
                .setLinearHeadingInterpolation(shakePose.getHeading(), pickupPoseOne.getHeading())
                .build();
        shakeFront = follower.pathBuilder()
                .addPath(new BezierLine(pickupPoseOne, shakePose))
                .setLinearHeadingInterpolation(pickupPoseOne.getHeading(), shakePose.getHeading())
                .build();

        moveOffPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, moveOffPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), moveOffPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:

                robotHardware.intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                robotHardware.intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);

                pathState = 2;




                break;
            case 2:
                pathTimer.resetTimer();

                if (robotHardware.shooterMotorBottom.getVelocity() == 1420) {



                    launchArtifacts();

                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1.5);
                    follower.followPath(shootToPickup);

                    robotHardware.intakeServoLeft.setPosition(SERVO_INTAKE_POS_LEFT);
                    robotHardware.intakeServoRight.setPosition(SERVO_INTAKE_POS_RIGHT);
                    robotHardware.intakeMotor.setPower(1);


                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {

                    for (int i = 0; i < 3; i++) {
                        indexingClass.indexArtifacts();
                        pulse();
                    }
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {

                    robotHardware.intakeMotor.setPower(1);

                    follower.followPath(shootArtifacts, 0.6, true);
                    pathState = 5;

                    pathTimer.resetTimer();
                }
                break;
            case 5:

                if (!follower.isBusy()) {
                    robotHardware.intakeMotor.setPower(-1);

                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1);

                    launchArtifacts();

                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1.5);


                    if (loops == 1) {
                        pathState = 6;
                    } else {
                        loops++;
                        pathState = 1;
                    }

                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(moveOffPath);

                    robotHardware.intakeMotor.setPower(0);
                    pathState = -1;
                    zero = true;
                }
        }
    }


    public void init() {
        robotHardware.init(hardwareMap);
        indexingClass.init(hardwareMap);
        shootingAndIntaking.init(hardwareMap);



        robotHardware.limelight.setPollRateHz(100);
        robotHardware.limelight.pipelineSwitch(0);

        pathTimer = new Timer();
        timer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        robotHardware.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.turretMotor.setPositionPIDFCoefficients(15);

        telemetry.addData("Init: ", "Complete");

    }

    public void start() {
        pathState = 1;
        robotHardware.limelight.start();
    }

    public void loop() {
         llResult = robotHardware.limelight.getLatestResult();

        double offset = llResult.getTx() * 2.57;
        int newTargetLL = robotHardware.turretMotor.getCurrentPosition() - (int) offset;



        if (pathState == 3 || pathState == 4 || pathState == 10) {
            indexingClass.indexArtifacts();
        }

        if (!zero) {
            robotHardware.shooterMotorTop.setVelocity(-1420);
            robotHardware.shooterMotorBottom.setVelocity(1420);

            robotHardware.turretMotor.setTargetPosition((llResult.isValid()) ? newTargetLL : 170);
            robotHardware.turretMotor.setPower(1);

        } else {
            robotHardware.shooterMotorTop.setVelocity(0);
            robotHardware.shooterMotorBottom.setVelocity(0);

            robotHardware.turretMotor.setTargetPosition(0);
            robotHardware.turretMotor.setPower(1);
        }

       telemetry.addData("State: ",pathState);

        follower.update();
        autonomousPathUpdate();
    }

    public void stop() {
        String otosEndPose = follower.poseTracker.getPose().getX() + " " + follower.poseTracker.getPose().getY() + " " + follower.poseTracker.getPose().getHeading();
        ReadWriteFile.writeFile(file, otosEndPose);
    }

    public void launchArtifacts() {
        robotHardware.intakeMotor.setPower(1);
        robotHardware.intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
        robotHardware.intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);

        pathTimer.resetTimer();
        while(pathTimer.getElapsedTimeSeconds() < 0.5);

        robotHardware.indexerServo.setPosition(INDEXER_SERVO_POS_A_EXTRA);
        indexingClass.emptyIndexerArray();
    }

    public void pulse() {
        robotHardware.intakeMotor.setPower(-1);
        timer.resetTimer();
        while (timer.getElapsedTimeSeconds() < 0.15) {
            indexingClass.indexArtifacts();
        }
        robotHardware.intakeMotor.setPower(1);

        timer.resetTimer();
        while (timer.getElapsedTimeSeconds() < 0.7) {
            indexingClass.indexArtifacts();
        }

    }

}
