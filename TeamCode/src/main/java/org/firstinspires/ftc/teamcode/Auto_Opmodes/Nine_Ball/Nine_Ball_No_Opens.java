package org.firstinspires.ftc.teamcode.Auto_Opmodes.Nine_Ball;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto_Opmodes.Nine_Ball.CLOSE.NineNoUnloadBlueClose;
import org.firstinspires.ftc.teamcode.Auto_Opmodes.Nine_Ball.CLOSE.NineNoUnloadRedClose;
import org.firstinspires.ftc.teamcode.Auto_Opmodes.Nine_Ball.FAR.NineNoUnloadBlueFar;
import org.firstinspires.ftc.teamcode.Auto_Opmodes.Nine_Ball.FAR.NineNoUnloadRedFar;


@Configurable
@Autonomous(name = "9 Ball - No Unload", group = "Nine Ball")
public class Nine_Ball_No_Opens extends SelectableOpMode {

   public Nine_Ball_No_Opens() {
       super("Select Nine Ball Auto - No Gate Open", s -> {
           s.add("Red Close Auto: ", NineNoUnloadRedClose::new);
           s.add("Blue Close Auto: ", NineNoUnloadBlueClose::new);
           s.add("Red Far Auto: ", NineNoUnloadRedFar::new);
           s.add("Blue Far Auto: ", NineNoUnloadBlueFar::new);
       });
   }

    


}
