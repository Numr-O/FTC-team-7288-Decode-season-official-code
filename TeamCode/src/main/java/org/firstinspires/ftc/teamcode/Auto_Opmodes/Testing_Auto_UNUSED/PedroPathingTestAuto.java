package org.firstinspires.ftc.teamcode.Auto_Opmodes.Testing_Auto_UNUSED;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.IndexingClass;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;
import org.firstinspires.ftc.teamcode.Used_Classes.Teleop_Only_Classes.TeleopShootingAndIntaking;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



public class PedroPathingTestAuto extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    IndexingClass indexingClass = new IndexingClass();
    TeleopShootingAndIntaking shootingAndIntaking = new TeleopShootingAndIntaking();
    Follower follower;
    Timer pathTimer;
    private int pathState;
    double pickupSpeed = 0.2 ;
    boolean startIndexing = true;


    double SERVO_INTAKE_POS_RIGHT = 0.34;
    double SERVO_INTAKE_POS_LEFT = 0.66;
    double SERVO_TRAVEL_POS_RIGHT = 0.5;
    double SERVO_TRAVEL_POS_LEFT = 0.5;


    private final Pose startPose = new Pose(123.058,121.979,Math.toRadians(45));
    private final Pose shootPose = new Pose(91.106,91.106,Math.toRadians(45));
    private final Pose pickupPoseOnePre = new Pose(96.28,83.766,0);
    private final Pose pickupPoseOnePost = new Pose(125.535,83.766,0);





    private PathChain startToShoot, shootToPickupPre, pickupPreToPickupPost, pickupPostToShoot;

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
                .addPath(new BezierLine(pickupPoseOnePost, shootPose))
                .setLinearHeadingInterpolation(pickupPoseOnePost.getHeading(), shootPose.getHeading())
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




                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 4) {
                        shootingAndIntaking.launchArtifacts();
                        indexingClass.emptyIndexerArray();
                    }
                    follower.followPath(shootToPickupPre);


                    robotHardware.intakeServoLeft.setPosition(SERVO_INTAKE_POS_LEFT);
                    robotHardware.intakeServoRight.setPosition(SERVO_INTAKE_POS_RIGHT);
                    robotHardware.intakeMotor.setPower(1);


                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {

                    follower.followPath(pickupPreToPickupPost, pickupSpeed, true);

                    pathTimer.resetTimer();
                    while(pathTimer.getElapsedTimeSeconds() < 1);

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
                    while (pathTimer.getElapsedTimeSeconds() < 4) {
                        shootingAndIntaking.launchArtifacts();
                        indexingClass.emptyIndexerArray();
                    }
                    pathState = -1;
                }
                break;
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

    public void start() {
        pathState = 0;
    }

    public void loop() {

        indexingClass.indexArtifacts();

        robotHardware.shooterMotorBottom.setVelocity(1050);
        robotHardware.shooterMotorTop.setVelocity(-1050);



        follower.update();
        autonomousPathUpdate();
    }
}
