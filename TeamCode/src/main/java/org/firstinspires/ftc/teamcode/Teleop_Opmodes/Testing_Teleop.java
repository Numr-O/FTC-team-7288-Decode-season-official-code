package org.firstinspires.ftc.teamcode.Teleop_Opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops.FlywheelTuner;
import org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops.Indexer_testing;
import org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops.LimeLightDistanceTesting;
import org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops.PIDTestingTeleop;
import org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops.TurretPossitionTestOpMode;
import org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops.TurretTesting;
import org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops.TurretNew;
import org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops.servoTest;


@Configurable
@TeleOp(name = "Testing Teleops ONLY", group = "Testing")
public class Testing_Teleop extends SelectableOpMode {



    public Testing_Teleop() {
        super("Select Testing Opmode: ", s -> {
            s.add("Flywheel Tuner - MOST USED!: ", FlywheelTuner::new);
            s.add("PID Testing Teleop: ", PIDTestingTeleop::new);
            s.add("Turret Testing Teleop: ", TurretTesting::new);
            s.add("LimeLight Distance Testing: ", LimeLightDistanceTesting::new);
            s.add("New Turret Test TeleOP: ", TurretNew::new);
            s.add("Servo Test TeleOP: ", servoTest::new);
            s.add("Turret Position Test TeleOP: ", TurretPossitionTestOpMode::new);
            s.add("Indexer Test TeleOp: ", Indexer_testing::new);
        });
    }






}
