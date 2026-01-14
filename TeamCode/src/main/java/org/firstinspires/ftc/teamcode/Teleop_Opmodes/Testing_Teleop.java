package org.firstinspires.ftc.teamcode.Teleop_Opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops.FlywheelTuner;
import org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops.PIDTestingTeleop;
import org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops.TurretTesting;
import org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops.LimeLightDistanceTesting;


@Configurable
@TeleOp(name = "Testing Teleops ONLY", group = "Testing")
public class Testing_Teleop extends SelectableOpMode {



    public Testing_Teleop() {
        super("Select Testing Opmode: ", s -> {
            s.add("PID Testing Teleop: ", PIDTestingTeleop::new);
            s.add("Turret Testing Teleop: ", TurretTesting::new);
            s.add("LimeLight Distance Testing: ", LimeLightDistanceTesting::new);
            s.add("Flywheel Tuner: ", FlywheelTuner::new);
        });
    }






}
