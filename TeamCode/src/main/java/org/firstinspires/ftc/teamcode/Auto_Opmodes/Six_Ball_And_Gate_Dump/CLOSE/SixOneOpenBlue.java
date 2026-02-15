package org.firstinspires.ftc.teamcode.Auto_Opmodes.Six_Ball_And_Gate_Dump.CLOSE;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.IndexingClass;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.TeleopShootingAndIntaking;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.File;

public class SixOneOpenBlue extends OpMode {
    File file = AppUtil.getInstance().getSettingsFile("endPose.txt");
    RobotHardware robotHardware = new RobotHardware();
    IndexingClass indexingClass = new IndexingClass();
    TeleopShootingAndIntaking shootingAndIntaking = new TeleopShootingAndIntaking();
    Follower follower;
    Timer pathTimer;
    private int pathState;
    double pickupSpeed = 0.29;
    boolean startIndexing = true;
    boolean zero = false;


    double INDEXER_SERVO_POS_A_EXTRA = 0.97;

    double SERVO_INTAKE_POS_RIGHT = 0.37;
    double SERVO_INTAKE_POS_LEFT = 0.63;
    double SERVO_TRAVEL_POS_RIGHT = 0.5;
    double SERVO_TRAVEL_POS_LEFT = 0.5;
    double SERVO_TRANSFER_POS_RIGHT = 0.63;
    double SERVO_TRANSFER_POS_LEFT = 0.4;


    private final Pose startPose = new Pose(22.07,121.15,Math.toRadians(135));
    private final Pose shootPose = new Pose(47.70,95.106,Math.toRadians(135));
    private final Pose pickupPoseOnePre = new Pose(46.10,85.76,Math.toRadians(180));
    private final Pose pickupPoseOnePost = new Pose(28,85.76,Math.toRadians(180));
    private final Pose pickupPoseOneCtrl = new Pose(31.2,73.11);
    private final Pose dumpPose = new Pose(18.09,74.91,Math.toRadians(180));

    private final Pose shootPoseTwo = new Pose(49.04,85 ,Math.toRadians(140));
    private final Pose pickupPoseTwoPre = new Pose(49,64.89,Math.toRadians(180));
    private final Pose pickupPoseTwoPost = new Pose(26,62.18,Math.toRadians(180));
    private final Pose dumpPoseTwo = new Pose(20,70,Math.toRadians(180));
    private final Pose pickupPoseTwoCtrl = new Pose(40,72);


    private final Pose shootPoseThree = new Pose(49,85,Math.toRadians(145));

    private final Pose moveOffPose = new Pose(40.98,80.09,Math.toRadians(115));





    private PathChain startToShoot, shootToPickupPre, pickupPreToPickupPost, dumpBalls ,dumpToShoot, moveOffPath;

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
        dumpBalls = follower.pathBuilder()
                .addPath(new BezierCurve(pickupPoseOnePost, pickupPoseOneCtrl, dumpPose))
                .setLinearHeadingInterpolation(pickupPoseOnePost.getHeading(), dumpPose.getHeading())
                .build();
        dumpToShoot = follower.pathBuilder()
                .addPath(new BezierLine(dumpPose, shootPoseTwo))
                .setLinearHeadingInterpolation(dumpPose.getHeading(), shootPoseTwo.getHeading())
                .build();
        moveOffPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseTwo, moveOffPose))
                .setLinearHeadingInterpolation(shootPoseTwo.getHeading(),moveOffPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robotHardware.intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                robotHardware.intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);

                follower.followPath(startToShoot, 0.7, true);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {

                    launchArtifacts();

                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1.5);
                    follower.followPath(shootToPickupPre);

                    robotHardware.intakeServoLeft.setPosition(SERVO_INTAKE_POS_LEFT);
                    robotHardware.intakeServoRight.setPosition(SERVO_INTAKE_POS_RIGHT);
                    robotHardware.intakeMotor.setPower(1);

                    zero = true;
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {

                    follower.followPath(pickupPreToPickupPost, pickupSpeed, true);



                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {

                    robotHardware.intakeMotor.setPower(0);

                    follower.followPath(dumpBalls);
                    pathState = 4;

                    zero = false;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    while(pathTimer.getElapsedTimeSeconds() < 1.5);


                    follower.followPath(dumpToShoot);
                    pathState = 5;

                    zero = false;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    launchArtifacts();

                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1);


                    pathState = 6;
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

        pathTimer = new Timer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    public void init_loop() {
        robotHardware.ledLight.setPosition(0.611);

        telemetry.addData("Selected Team: ", "BLUE");
        telemetry.addData("Init: ", "Complete");


        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
    }

    public void start() {
        pathState = 0;
    }

    public void loop() {
        if (pathState == 2 || pathState == 3) {
            indexingClass.indexArtifacts();
        }



        if (!zero) {
            robotHardware.shooterMotorBottom.setVelocity(267, AngleUnit.DEGREES);
            robotHardware.shooterMotorTop.setVelocity(-267, AngleUnit.DEGREES);
        } else {
            robotHardware.shooterMotorTop.setVelocity(0);
            robotHardware.shooterMotorBottom.setVelocity(0);
        }


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

}
