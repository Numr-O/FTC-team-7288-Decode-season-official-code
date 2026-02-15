package org.firstinspires.ftc.teamcode.Auto_Opmodes.Nine_Ball_Two_Gate_Open;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto_Opmodes.Nine_Ball_Two_Gate_Open.CLOSE.NineTwoUnloadRed;
import org.firstinspires.ftc.teamcode.Auto_Opmodes.Nine_Ball_Two_Gate_Open.CLOSE.NineTwoUnload_Blue;

@Autonomous(name = " CLOSE ONLY - 9 Ball - 2 Unload", group = "9 Two")
public class Nine_Two_Unload extends SelectableOpMode {

    public Nine_Two_Unload() {
        super("Select Nine Ball Auto - TWO Gate Opens", s -> {
            s.add( "RED" ,NineTwoUnloadRed::new);
            s.add("BLUE", NineTwoUnload_Blue::new);
        });
    }



}
