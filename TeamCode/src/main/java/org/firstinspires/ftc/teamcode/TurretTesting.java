package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mainCode.RobotHardware;

@TeleOp
public class TurretTesting extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    int turretPos;

    public void init() {
        robotHardware.init(hardwareMap);
        turretPos = robotHardware.turretMotor.getCurrentPosition();


    }

    public void loop() {
        if (gamepad1.aWasReleased()) {
            turretPos++;
        } else if (gamepad1.bWasReleased()) {
            turretPos--;
        }
        robotHardware.turretMotor.setTargetPosition(turretPos);

        telemetry.addData("Expected Turret Position: ", turretPos);
        telemetry.addData("Current Turret Position: ", robotHardware.turretMotor.getCurrentPosition());
    }
}
