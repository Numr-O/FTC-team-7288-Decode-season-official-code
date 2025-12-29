package org.firstinspires.ftc.teamcode.Auto_Opmodes.Testing_Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;

@Autonomous
public class PedroPathingTestAuto extends OpMode {
    RobotHardware robotHardware = new RobotHardware();


    public void init() {
        robotHardware.init(hardwareMap);



    }

    public void loop() {

    }
}
