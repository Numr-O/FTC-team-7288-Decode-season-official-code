package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mainCode.RobotHardware;

@Configurable
@TeleOp
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
        telemetry.addData("indexer position: ",robotHardware.indexerServo.getPosition());
        LLResult llResult = robotHardware.limelight.getLatestResult();
        double error = llResult.getTx();


        controller.setPID(kP,kI,kD);
        double pid = controller.calculate(-error, setpoint);
        double ff = kF;

        double power = pid + ff;
        //robotHardware.turretMotor.setPower(power);

        panelsTelemetry.addData("Error", error);
        panelsTelemetry.addData("Setpoint", setpoint);
        telemetry.addData("Error", error);
        telemetry.addData("Power",power);
        panelsTelemetry.addData("Power", power);
        panelsTelemetry.update();
    }
}
