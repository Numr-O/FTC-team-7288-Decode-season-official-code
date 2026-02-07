package org.firstinspires.ftc.teamcode.Teleop_Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.IndexingClass;
import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;

@TeleOp(name = "Loading", group = "Comp TeleOp")
public class LoadingTeleOP extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    IndexingClass indexingClass = new IndexingClass();

    public void init() {
        robotHardware.init(hardwareMap);
        indexingClass.init(hardwareMap);
    }

    public void loop() {
        indexingClass.indexArtifacts();
        telemetry.addData("Distance: ",robotHardware.distanceSensor.getDistance(DistanceUnit.MM));
    }


}
