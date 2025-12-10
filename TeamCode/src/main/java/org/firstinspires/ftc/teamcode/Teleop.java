/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mainCode.IndexerShootingAndIntake;
import org.firstinspires.ftc.teamcode.mainCode.LimeLightTrackingAndDistance;
import org.firstinspires.ftc.teamcode.mainCode.MecanumDriveConsentric;
import org.firstinspires.ftc.teamcode.mainCode.RobotHardware;


@TeleOp
public class Teleop extends OpMode {

    RobotHardware robothwde = new RobotHardware();
    double headingOffset;

    double SERVOINTAKEPOSRIGHT = 0.55;
    double SERVOINTAKEPOSLEFT = 0.47;
    double SERVOTRAVELPOSRIGHT = 0.73;
    double SERVOTRAVELPOSLEFT = 0.30;
    double SERVOTRANSFERPOSRIGHT = 0.83;
    double SERVOTRANSFERPOSLEFT = 0.2;
    double POSA = 0.9;
    double POSB = 0.5;
    double POSC = 0.09;
    boolean priority = false;
    double motorvel;
    String selectedTeam;

    MecanumDriveConsentric mecanumDriveConsentric;
    LimeLightTrackingAndDistance limeLightTrackingAndDistance;
    IndexerShootingAndIntake indexerShootingAndIntake;
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robothwde.init(hardwareMap);

        mecanumDriveConsentric = new MecanumDriveConsentric(robothwde.frontLeftMotor, robothwde.backLeftMotor, robothwde.frontRightMotor, robothwde.backRightMotor, robothwde.imu);

        limeLightTrackingAndDistance = new LimeLightTrackingAndDistance(robothwde.turretMotor);

        Object[] hardwareList = {robothwde.indexerServo,robothwde.intakeServoLeft,robothwde.intakeServoRight,robothwde.shooterMotorTop,robothwde.shooterMotorBottom,robothwde.intakeMotor,robothwde.colorPosA,robothwde.colorPosB,robothwde.colorPosC};
        indexerShootingAndIntake = new IndexerShootingAndIntake(hardwareList);


        robothwde.limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        //This is the setup for the limelight A3 camera's pipeline
        robothwde.limelight.pipelineSwitch(0);

    }

        /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        if (gamepad1.a) {
            robothwde.limelight.pipelineSwitch(0);
            selectedTeam = "Red";
        } else if (gamepad1.b) {
            robothwde.limelight.pipelineSwitch(1);
            selectedTeam = "Blue";
        }
        telemetry.addData("Selected Team: ", selectedTeam);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        robothwde.limelight.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.start) {
            headingOffset = -robothwde.imu.getAngularOrientation().firstAngle;
        }

        LLResult llResult = robothwde.limelight.getLatestResult();
        limeLightTrackingAndDistance.setLLResult(llResult);

        double distanceOne = limeLightTrackingAndDistance.distanceToTarget();
        double distanceTwo = limeLightTrackingAndDistance.distanceToTargetTwo();
        limeLightTrackingAndDistance.trackAprilTagWithPID();

        mecanumDriveConsentric.controlerDrive(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,headingOffset);



        telemetry.addData("TX", llResult.getTx());
        telemetry.addData("TY", llResult.getTy());
        telemetry.addData("TA", llResult.getTa());
        telemetry.addData("Distance One: ", distanceOne);
        telemetry.addData("Distance Two: ", distanceTwo);
        telemetry.addData("test", robothwde.colorPosA.getDistance(DistanceUnit.MM));




        priority = gamepad1.right_trigger > 0.5;
        indexerShootingAndIntake.ShootingOrIndexingArtifacts(limeLightTrackingAndDistance.calculateRPMForShooter(), gamepad1.right_trigger>0.5);

        boolean leftTrigger = gamepad1.left_trigger > 0.5;
        boolean leftBack = gamepad1.left_bumper;

        if (leftTrigger && !leftBack && !priority) {
            robothwde.intakeMotor.setPower(1);
            robothwde.intakeServoRight.setPosition(SERVOINTAKEPOSRIGHT);
            robothwde.intakeServoLeft.setPosition(SERVOINTAKEPOSLEFT);
        } else if (!leftTrigger && leftBack && !priority) {
            robothwde.intakeMotor.setPower(-1);
            robothwde.intakeServoRight.setPosition(SERVOINTAKEPOSRIGHT);
            robothwde.intakeServoLeft.setPosition(SERVOINTAKEPOSLEFT);
        } else if (!leftTrigger && !leftBack  && !priority) {
            robothwde.intakeMotor.setPower(0);
            robothwde.intakeServoRight.setPosition(SERVOTRAVELPOSRIGHT);
            robothwde.intakeServoLeft.setPosition(SERVOTRAVELPOSLEFT);
        }

        telemetry.addData("RPM", limeLightTrackingAndDistance.calculateRPMForShooter());

//        if (gamepad1.right_trigger > 0.5) {
//            robothwde.shooterMotorTop.setVelocity(-limeLightTrackingAndDistance.calculateRPMForShooter());
//            robothwde.shooterMotorBottom.setVelocity(limeLightTrackingAndDistance.calculateRPMForShooter());
//        } else {
//            robothwde.shooterMotorTop.setVelocity(0);
//            robothwde.shooterMotorBottom.setVelocity(0);
//        }


        telemetry.addData("top shooter velocity", robothwde.shooterMotorTop.getVelocity());
        telemetry.addData("bottom shooter velocity", robothwde.shooterMotorBottom.getVelocity());
        telemetry.addData("Are Shooter Motors At Speed: ", indexerShootingAndIntake.areShooterMotorsAtSpeed);
        telemetry.addData("Shooter Case", indexerShootingAndIntake.getShooterState());
        telemetry.addData("Indexing case: ", indexerShootingAndIntake.getIndexingState());
        telemetry.addData("Array", indexerShootingAndIntake.artifactAtIndexerPositions[0] + " " + indexerShootingAndIntake.artifactAtIndexerPositions[1] + " " + indexerShootingAndIntake.artifactAtIndexerPositions[2]);




        if (gamepad2.dpad_up) {
            robothwde.indexerServo.setPosition(0.91);
        } else if (gamepad2.dpad_right) {
            robothwde.indexerServo.setPosition(0.5);
        } else if (gamepad2.dpad_down) {
            robothwde.indexerServo.setPosition(0.09);
        }


    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
