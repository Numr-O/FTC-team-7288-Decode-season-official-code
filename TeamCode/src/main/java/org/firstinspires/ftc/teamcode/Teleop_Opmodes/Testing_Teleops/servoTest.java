package org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;

@Configurable
public class servoTest extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    double SERVO_INTAKE_POS_RIGHT = 0.34;
    double SERVO_INTAKE_POS_LEFT = 0.66;
    double SERVO_TRAVEL_POS_RIGHT = 0.5;
    double SERVO_TRAVEL_POS_LEFT = 0.5;
    double SERVO_TRANSFER_POS_RIGHT = 0.63;
    double SERVO_TRANSFER_POS_LEFT = 0.4;

    public void init() {
        robotHardware.init(hardwareMap);
    }


    public void loop() {
        boolean rightTrigger = gamepad1.right_trigger > 0.5;
        boolean leftTrigger = gamepad1.left_trigger > 0.5;


        if (rightTrigger && !leftTrigger) {
            robotHardware.intakeServoRight.setPosition(SERVO_INTAKE_POS_RIGHT);
        } else if (!rightTrigger && leftTrigger) {
            robotHardware.intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
        } else {
            robotHardware.intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
        }


    }

}
