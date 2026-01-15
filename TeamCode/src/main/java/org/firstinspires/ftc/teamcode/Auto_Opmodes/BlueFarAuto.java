package org.firstinspires.ftc.teamcode.Auto_Opmodes;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.IndexingClass;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.TeleopShootingAndIntaking;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Blue Far Auto", group = "FAR AUTO")
public class BlueFarAuto extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    IndexingClass indexingClass = new IndexingClass();
    TeleopShootingAndIntaking shootingAndIntaking = new TeleopShootingAndIntaking();
    Follower follower;
    Timer pathTimer;
    Timer timer;
    private int pathState;
    double pickupSpeed = 0.25 ;
    boolean startIndexing = true;
    boolean zero;

    double INDEXER_SERVO_POS_A_EXTRA = 0.97;

    double SERVO_INTAKE_POS_RIGHT = 0.34;
    double SERVO_INTAKE_POS_LEFT = 0.66;
    double SERVO_TRAVEL_POS_RIGHT = 0.5;
    double SERVO_TRAVEL_POS_LEFT = 0.5;
    double SERVO_TRANSFER_POS_RIGHT = 0.63;
    double SERVO_TRANSFER_POS_LEFT = 0.4;


    private final Pose startPose = new Pose(47.71,8.20,Math.toRadians(90));
    private final Pose shootPose = new Pose(58.88,18.13,Math.toRadians(119));
    private final Pose pickupPoseOnePre = new Pose(51.81,35.83,Math.toRadians(180));
    private final Pose pickupPoseOnePost = new Pose(28.48,38.83,Math.toRadians(180));
    private final Pose moveOffPose = new Pose(56.77,26.98,Math.toRadians(119));
    private final Pose shootPoseTwo = new Pose(58.88,18.13,Math.toRadians(115));




    private PathChain startToShoot, shootToPickupPre, pickupPreToPickupPost, pickupPostToShoot,moveOffPath;

    public void buildPaths() {

        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootToPickupPre = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPoseOnePre))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPoseOnePre.getHeading())
                .build();
        pickupPreToPickupPost = follower.pathBuilder()
                .addPath(new BezierLine(pickupPoseOnePre, pickupPoseOnePost))
                .setLinearHeadingInterpolation(pickupPoseOnePre.getHeading(), pickupPoseOnePost.getHeading())
                .build();
        pickupPostToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickupPoseOnePost, shootPoseTwo))
                .setLinearHeadingInterpolation(pickupPoseOnePost.getHeading(), shootPoseTwo.getHeading())
                .build();
        moveOffPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, moveOffPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), moveOffPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(startToShoot, 0.7, true);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {

                    robotHardware.intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
                    robotHardware.intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);
                    robotHardware.intakeMotor.setPower(1);

                    pathTimer.resetTimer();
                    while(pathTimer.getElapsedTimeSeconds() < 1);

                    robotHardware.indexerServo.setPosition(INDEXER_SERVO_POS_A_EXTRA);
                    indexingClass.emptyIndexerArray();

                    pathTimer.resetTimer();
                    while(pathTimer.getElapsedTimeSeconds() < 3);


                    follower.followPath(shootToPickupPre, 0.8, true);


                    robotHardware.intakeServoLeft.setPosition(SERVO_INTAKE_POS_LEFT);
                    robotHardware.intakeServoRight.setPosition(SERVO_INTAKE_POS_RIGHT);
                    robotHardware.intakeMotor.setPower(1);

                    timer.resetTimer();
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy() || timer.getElapsedTimeSeconds() > 2) {

                    follower.followPath(pickupPreToPickupPost, pickupSpeed, true);

                    pathTimer.resetTimer();
                    while(pathTimer.getElapsedTimeSeconds() < 1);

                    timer.resetTimer();
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {


                    follower.followPath(pickupPostToShoot, 0.75, true);
                    pathState = 4;

                }
                break;
            case 4:
                if (!follower.isBusy()) {

                    robotHardware.intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);
                    robotHardware.intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                    robotHardware.intakeMotor.setPower(0);

                    pathTimer.resetTimer();
                    while(pathTimer.getElapsedTimeSeconds() < 0.5);

                    robotHardware.intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
                    robotHardware.intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);
                    robotHardware.intakeMotor.setPower(1);

                    pathTimer.resetTimer();
                    while(pathTimer.getElapsedTimeSeconds() < 1);

                    robotHardware.indexerServo.setPosition(INDEXER_SERVO_POS_A_EXTRA);
                    indexingClass.emptyIndexerArray();

                    while(pathTimer.getElapsedTimeSeconds() < 3);

                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(moveOffPath, 0.75, true);
                    pathState = -1;

                    zero = true;


                }
                break;
        }
    }


    public void init() {
        robotHardware.init(hardwareMap);
        indexingClass.init(hardwareMap);
        shootingAndIntaking.init(hardwareMap);

        robotHardware.limelight.setPollRateHz(100);
        robotHardware.limelight.pipelineSwitch(1);

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
        pathState = 0;
        robotHardware.limelight.start();
    }

    public void loop() {
        LLResult llResult = robotHardware.limelight.getLatestResult();

        if (pathState == 2 || pathState == 3) {
            indexingClass.indexArtifacts();
        }

        if (!zero) {
            robotHardware.shooterMotorTop.setVelocity(-338.5, AngleUnit.DEGREES);
            robotHardware.shooterMotorBottom.setVelocity(338.5, AngleUnit.DEGREES);
        } else {
            robotHardware.shooterMotorTop.setVelocity(0);
            robotHardware.shooterMotorBottom.setVelocity(0);
            robotHardware.intakeMotor.setPower(0);
        }



        follower.update();
        autonomousPathUpdate();
    }
}
