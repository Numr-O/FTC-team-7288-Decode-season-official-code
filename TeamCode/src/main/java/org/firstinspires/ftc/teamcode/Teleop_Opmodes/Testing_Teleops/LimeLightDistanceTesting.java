package org.firstinspires.ftc.teamcode.Teleop_Opmodes.Testing_Teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes.RobotHardware;

import java.util.List;

@Configurable
public class LimeLightDistanceTesting extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    Limelight3A limelight3A;

    public void init() {
        robotHardware.init(hardwareMap);
        limelight3A = hardwareMap.get(Limelight3A.class, "LimeLight");

        limelight3A.setPollRateHz(100);
        limelight3A.pipelineSwitch(0);
    }

    public void start() {limelight3A.start();}

    public void loop() {

        robotHardware.indexerServo.setPosition(0.5);

        LLResult llResult = limelight3A.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        for(LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == 24) {
                telemetry.addData("Distance: ", fiducial.getCameraPoseTargetSpace().getPosition().z);
            }
        }


        telemetry.addData("Distance: ", llResult.getBotpose().getPosition().z);

    }

}
