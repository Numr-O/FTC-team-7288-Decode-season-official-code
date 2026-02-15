package org.firstinspires.ftc.teamcode.Auto_Opmodes.Nine_Ball_One_Gate_Open;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auto_Opmodes.Nine_Ball_One_Gate_Open.CLOSE.NineOneUnloadBlue;
import org.firstinspires.ftc.teamcode.Auto_Opmodes.Nine_Ball_One_Gate_Open.CLOSE.NineOneUnloadRed;

@Autonomous(name = "CLOSE ONLY - 9 Ball - 1 Unload", group = "9 One")
public class Nine_One_Unload extends SelectableOpMode {

    public Nine_One_Unload() {
        super("Select Nine Ball Auto - TWO Gate Opens", s -> {
            s.add( "RED" , NineOneUnloadRed::new);
            s.add("BLUE", NineOneUnloadBlue::new);
        });
    }
}
