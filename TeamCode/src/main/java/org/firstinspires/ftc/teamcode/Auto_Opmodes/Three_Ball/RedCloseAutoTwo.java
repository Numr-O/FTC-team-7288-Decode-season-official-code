package org.firstinspires.ftc.teamcode.Auto_Opmodes.Three_Ball;


import com.pedropathing.follower.Follower;
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


@Autonomous(name = "Red Close Auto Two", group = "CLOSE AUTO")
public class RedCloseAutoTwo extends OpMode {
    File file = AppUtil.getInstance().getSettingsFile("endPose.txt");
    RobotHardware robotHardware = new RobotHardware();
    IndexingClass indexingClass = new IndexingClass();
    TeleopShootingAndIntaking shootingAndIntaking = new TeleopShootingAndIntaking();
    Follower follower;
    Timer pathTimer;
    private int pathState;
    double pickupSpeed = 0.28 ;
    boolean startIndexing = true;
    boolean zero = false;


    double INDEXER_SERVO_POS_A_EXTRA = 0.97;

    double SERVO_INTAKE_POS_RIGHT = 0.34;
    double SERVO_INTAKE_POS_LEFT = 0.66;
    double SERVO_TRAVEL_POS_RIGHT = 0.5;
    double SERVO_TRAVEL_POS_LEFT = 0.5;
    double SERVO_TRANSFER_POS_RIGHT = 0.63;
    double SERVO_TRANSFER_POS_LEFT = 0.4;


    private final Pose startPose = new Pose(123.058,121.979,Math.toRadians(45));
    private final Pose shootPose = new Pose(91.106,91.106,Math.toRadians(45));
    private final Pose pickupPoseOnePre = new Pose(96.28,83.766,0);
    private final Pose pickupPoseOnePost = new Pose(118.535,83.766,0);
    private final Pose shootPoseTwo = new Pose(91.106,91.106,Math.toRadians(45));
    private final Pose moveOffPose = new Pose(103.19,80.09,Math.toRadians(60));





    private PathChain startToShoot, shootToPickupPre, pickupPreToPickupPost, pickupPostToShoot, moveOffPath;

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
                .setLinearHeadingInterpolation(shootPose.getHeading(),moveOffPose.getHeading())
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
                    while (pathTimer.getElapsedTimeSeconds() < 3);
                    follower.followPath(shootToPickupPre);


                    robotHardware.intakeServoLeft.setPosition(SERVO_INTAKE_POS_LEFT);
                    robotHardware.intakeServoRight.setPosition(SERVO_INTAKE_POS_RIGHT);
                    robotHardware.intakeMotor.setPower(1);


                    pathState = 5;
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



                    robotHardware.intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);
                    robotHardware.intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                    robotHardware.intakeMotor.setPower(0);

                    follower.followPath(pickupPostToShoot);
                    pathState = 4;


                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    robotHardware.intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
                    robotHardware.intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);
                    robotHardware.intakeMotor.setPower(1);

                    pathTimer.resetTimer();
                    while(pathTimer.getElapsedTimeSeconds() < 1);

                    robotHardware.indexerServo.setPosition(INDEXER_SERVO_POS_A_EXTRA);
                    indexingClass.emptyIndexerArray();


                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 3);

                    pathState = 5;
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(moveOffPath);
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

        telemetry.addData("Init: ", "Complete");
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
            robotHardware.intakeMotor.setPower(0);
        }


        follower.update();
        autonomousPathUpdate();
    }

    public void stop() {
        String otosEndPose = follower.poseTracker.getPose().getX() + " " + follower.poseTracker.getPose().getY() + " " + follower.poseTracker.getPose().getHeading();
        ReadWriteFile.writeFile(file, otosEndPose);
    }
}
