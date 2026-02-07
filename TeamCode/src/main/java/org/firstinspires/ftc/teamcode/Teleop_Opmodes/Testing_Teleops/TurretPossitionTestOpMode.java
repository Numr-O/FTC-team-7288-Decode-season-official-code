package org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.TurretPositionUpdater;

import java.io.File;

@Configurable
public class TurretPossitionTestOpMode extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    TurretPositionUpdater turretPositionUpdater = new TurretPositionUpdater();
    File file;
    String[] fileContents;
    Double[] initialRobotPose;
    Double initialHeading;




    public void init() {
        fileContents = ReadWriteFile.readFile(file = AppUtil.getInstance().getSettingsFile("endPose.txt")).split(" ");
        initialRobotPose[0] = Double.parseDouble(fileContents[0]); initialRobotPose[1] = Double.parseDouble(fileContents[1]); initialHeading = Double.parseDouble(fileContents[2]);

        robotHardware.init(hardwareMap);
        turretPositionUpdater.init(initialRobotPose, 0.0, 0);
        robotHardware.limelight.setPollRateHz(100);
        robotHardware.limelight.pipelineSwitch(0);

        SparkFunOTOS.Pose2D initialPose = new SparkFunOTOS.Pose2D(initialRobotPose[0],initialRobotPose[1],initialHeading);
        robotHardware.sparkFunOTOS.setPosition(initialPose);
    }

    public void start() {
        robotHardware.limelight.start();
    }

    public void loop() {


        telemetry.addData("Current Pos: ", robotHardware.turretMotor.getCurrentPosition());
        telemetry.addData("Current Conversion Old: ", robotHardware.turretMotor.getCurrentPosition() / 2.57);
        telemetry.addData("Current Conversion New: ", robotHardware.turretMotor.getCurrentPosition() / 1.712);
        telemetry.addData("LL not times Kt: ", turretPositionUpdater.updatePosition(robotHardware.limelight.getLatestResult().getTx(),robotHardware.imu.getAngularOrientation().firstAngle,0,0) / 2.57);
        telemetry.addData("LL times Kt: ", turretPositionUpdater.updatePositionTwo(robotHardware.limelight.getLatestResult().getTx(),robotHardware.imu.getAngularOrientation().firstAngle,0,0) / 2.57);
        telemetry.addData("No LL: ", turretPositionUpdater.updatePositionOfTurret(robotHardware.imu.getAngularOrientation().firstAngle,0,0) / 2.57);
        telemetry.addData("Tx: ", robotHardware.limelight.getLatestResult().getTx());



    }


}
