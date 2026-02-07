package org.firstinspires.ftc.teamcode.Auto_Opmodes.Six_Ball;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auto_Opmodes.Six_Ball.CLOSE.BlueCloseAutoSixBall;
import org.firstinspires.ftc.teamcode.Auto_Opmodes.Six_Ball.CLOSE.RedCloseAutoSixBall;
import org.firstinspires.ftc.teamcode.Auto_Opmodes.Six_Ball.FAR.BlueFarAutoSixBall;
import org.firstinspires.ftc.teamcode.Auto_Opmodes.Six_Ball.FAR.RedFarAutoSixBall;


@Configurable
@Autonomous(name = "Six Ball: ", group = "Six Ball")
public class Selectable_Six_Ball extends SelectableOpMode {

   public Selectable_Six_Ball() {
       super("Select Six Ball Auto - No Gate Dump", s -> {
           s.add("Red Close Auto: ", RedCloseAutoSixBall::new);
           s.add("Blue Close Auto: ", BlueCloseAutoSixBall::new);
           s.add("Red Far Auto: ", RedFarAutoSixBall::new);
           s.add("Blue Far Auto: ", BlueFarAutoSixBall::new);
       });
   }




}
