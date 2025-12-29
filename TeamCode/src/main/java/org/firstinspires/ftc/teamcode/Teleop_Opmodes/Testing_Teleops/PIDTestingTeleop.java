package org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;

@Configurable
public class PIDTestingTeleop extends OpMode {
    TelemetryManager panelsTelemetry;
    RobotHardware robotHardware = new RobotHardware();
    public PIDController controller;
    public static double kP = 0.0305,kI = 0.005,kD = 0.001;
    public static double kF = 0.03;




    public double intakeServoLeft = 0.48, intakeServoRight = 0.55;
    public double travelLeft = 0.37;
    public double travelRight = 0.66;
    public double intermediateLeft = 0.25;
    public double intermediateRight = 0.78;
    public double transferLeft = 0.2;
    public double transferRight = 0.83;

    public int setpoint = 0;

    @Override
    public void init () {
        robotHardware.init(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        controller = new PIDController(kP,kI,kD);
        robotHardware.limelight.setPollRateHz(100);
        robotHardware.limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        robotHardware.limelight.start();
    }


    @Override
    public void loop() {
        robotHardware.shooterMotorBottom.setVelocity(1000);
        robotHardware.shooterMotorTop.setVelocity(-1000 );



        robotHardware.intakeMotor.setPower(-1);

        if (gamepad1.aWasReleased()) {
            robotHardware.indexerServo.setPosition(0.95);
        } else if (gamepad1.bWasReleased()) {
            robotHardware.indexerServo.setPosition(0.5);
        } else if (gamepad1.xWasReleased()) {
            robotHardware.indexerServo.setPosition(0.11);
        }

        if (gamepad1.right_trigger > 0.5) {
            robotHardware.intakeServoLeft.setPosition(0.4);
            robotHardware.intakeServoRight.setPosition(0.63);
        } else {
            robotHardware.intakeServoLeft.setPosition(0.66);
            robotHardware.intakeServoRight.setPosition(0.34);

        }


    }
}
