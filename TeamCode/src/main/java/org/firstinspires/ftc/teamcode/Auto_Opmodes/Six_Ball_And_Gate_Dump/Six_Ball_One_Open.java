package org.firstinspires.ftc.teamcode.Auto_Opmodes.Six_Ball_And_Gate_Dump;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto_Opmodes.Six_Ball_And_Gate_Dump.CLOSE.SixOneOpenBlue;
import org.firstinspires.ftc.teamcode.Auto_Opmodes.Six_Ball_And_Gate_Dump.CLOSE.SixOneOpenRed;

@Autonomous(name = "CLOSE ONLY - 6 Ball - 1 Open", group = "6 One")
public class Six_Ball_One_Open extends SelectableOpMode {

    public Six_Ball_One_Open() {
        super("CLOSE ONLY - Select Six Ball Auto - ONE Gate Dump", s -> {
            s.add("Red Close Auto: ", SixOneOpenRed::new);
            s.add("Blue Close Auto: ", SixOneOpenBlue::new);
        });
    }

}
