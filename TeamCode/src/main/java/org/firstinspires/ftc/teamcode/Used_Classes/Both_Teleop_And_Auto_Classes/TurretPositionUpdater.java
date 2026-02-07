package org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class TurretPositionUpdater {

    File file;
    String[] fileContents;
    SimpleMatrix initialVector = new SimpleMatrix(2,1);
    SimpleMatrix currentVector = new SimpleMatrix(2,1);

    Double[] blueGoalPose = {16.0,131.0};
    Double[] redGoalPose = {128.0,131.0};
    Double[] selectedTeamPose = {0.0, 0.0};
    Double[] initialRobotPose = new Double[2];
    Double initialHeading;
    Double initialAngle;
    Double currentHeading;

    double times;
    double Kt = 2.57;
    double multiplier = 1;

    public void init(Double[] initialRobotPose, Double initialHeading, int teamPipeLine) {
        this.initialRobotPose = initialRobotPose;
        this.initialHeading = initialHeading;
        this.initialAngle = initialAngle;


        if (teamPipeLine == 1) {
            selectedTeamPose = blueGoalPose;
        } else {
            selectedTeamPose = redGoalPose;
        }


        initialVector.set(0,0,selectedTeamPose[0]-initialRobotPose[0]);
        initialVector.set(1,0,selectedTeamPose[1]-initialRobotPose[1]);

        currentVector.set(0,0,selectedTeamPose[0]-initialRobotPose[0]);
        currentVector.set(1,0,selectedTeamPose[1]-initialRobotPose[1]);


    }

    public int updatePosition(double txOffset, double imuAngle, double x, double y) {

        currentVector.set(0,0,selectedTeamPose[0]-x);
        currentVector.set(1,0,selectedTeamPose[1]-y);



        double translationalAngle = Math.acos(initialVector.dot(currentVector) / (initialVector.normF() * currentVector.normF()));
        if (x - initialRobotPose[0] < 0) {
            multiplier = 1;
        } else {
            multiplier = -1;
        }


        double angle = multiplier * translationalAngle;
        double angleRotation = angle + imuAngle;
        angleRotation = Math.toDegrees(angleRotation);
        double trueTxOffset = (txOffset < 3) ? 2 * txOffset : txOffset ;

        return (int) ((Kt * angleRotation) - trueTxOffset);
    }


    public int updatePositionTwo(double txOffset, double imuAngle, double x, double y) {

        currentVector.set(0,0,selectedTeamPose[0]-x);
        currentVector.set(1,0,selectedTeamPose[1]-y);



        double translationalAngle = Math.acos(initialVector.dot(currentVector) / (initialVector.normF() * currentVector.normF()));
        if (x - initialRobotPose[0] < 0) {
            multiplier = 1;
        } else {
            multiplier = -1;
        }


        double angle = multiplier * translationalAngle;
        double angleRotation = angle + imuAngle;
        angleRotation = Math.toDegrees(angleRotation);
        double trueTxOffset = (txOffset < 3) ? 2 * txOffset : txOffset ;

        return (int) (Kt * (angleRotation - trueTxOffset));
    }

    public int updatePositionOfTurret(double imuAngle, double x, double y) {

        currentVector.set(0,0,selectedTeamPose[0]-x);
        currentVector.set(1,0,selectedTeamPose[1]-y);



        double translationalAngle = Math.acos(initialVector.dot(currentVector) / (initialVector.normF() * currentVector.normF()));
        if (x - initialRobotPose[0] < 0) {
            multiplier = 1;
        } else {
            multiplier = -1;
        }


        double angle = multiplier * translationalAngle;
        double angleRotation = angle + imuAngle;
        angleRotation = Math.toDegrees(angleRotation);


        return (int) (Kt * angleRotation);
    }

    public double distanceToGoal () {
        return 2.54 * currentVector.normF();
    }


}
