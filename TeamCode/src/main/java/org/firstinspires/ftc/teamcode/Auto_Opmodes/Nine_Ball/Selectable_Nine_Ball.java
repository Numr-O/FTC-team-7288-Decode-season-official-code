package org.firstinspires.ftc.teamcode.Auto_Opmodes.Nine_Ball;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auto_Opmodes.Nine_Ball.CLOSE.RedCloseAutoNineBall;


@Configurable
@Autonomous(name = "Nine Ball: ", group = "Nine Ball")
public class Selectable_Nine_Ball extends SelectableOpMode {

   public Selectable_Nine_Ball() {
       super("Select Six Ball Auto - No Gate Dump", s -> {
           s.add("Red Close Auto: ", RedCloseAutoNineBall::new);
       });
   }

    


}
