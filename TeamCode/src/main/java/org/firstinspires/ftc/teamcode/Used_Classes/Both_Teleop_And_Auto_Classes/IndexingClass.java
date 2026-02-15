package org.firstinspires.ftc.teamcode.Used_Classes.Both_Teleop_And_Auto_Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IndexingClass {
    RobotHardware robotHardware = new RobotHardware();
    ElapsedTime timer;

    //--------- Variables --------

    public boolean[] indexerPositions = {false, false, false};
    public boolean green;



    //-------- Constants ----------------

    double INDEXER_SERVO_POS_A = 0.9;
    double INDEXER_SERVO_POS_B = 0.5;
    double INDEXER_SERVO_POS_C = 0.11;

    // --------- States -----------------
    public enum IndexerStates {
        INTERMEDIATE_STATE,
        INDEX_TO_A,
        INDEX_TO_B,
        INDEX_TO_C,
        INDEXER_FULL
    }
    public IndexerStates indexerStates = IndexerStates.INTERMEDIATE_STATE;

    // ---------- Initializing -------------
    public void init(HardwareMap hardwareMap) {
        robotHardware.init(hardwareMap);

        timer = new ElapsedTime();
    }


    // ---------- Functions ------------

    public boolean doesPosAHaveArtifact() {
        return robotHardware.colorPosA.getDistance(DistanceUnit.MM) < 59 && robotHardware.colorPosA.getLightDetected() > 0.1;
    }

    public boolean doesPosBHaveArtifact() {
        return robotHardware.colorPosB.getDistance(DistanceUnit.MM) < 59 && robotHardware.colorPosB.getLightDetected() > 0.1;
    }

    public boolean doesPosCHaveArtifact() {
        return robotHardware.colorPosC.getDistance(DistanceUnit.MM) < 59 && robotHardware.colorPosC.getLightDetected() > 0.1;
    }

    public boolean isBallInIndexer() {
        return robotHardware.breakBeam.getState();
    }

    public boolean distanceSensor() {
        return robotHardware.distanceSensor.getDistance(DistanceUnit.MM) < 60;
    }


    public void indexArtifacts() {
        switch (indexerStates) {
            case INTERMEDIATE_STATE:
                if (!indexerPositions[0]) {
                    indexerStates = IndexerStates.INDEX_TO_A;
                } else if (!indexerPositions[1]) {
                    indexerStates = IndexerStates.INDEX_TO_B;
                } else if (!indexerPositions[2]) {
                    indexerStates = IndexerStates.INDEX_TO_C;
                } else {
                    indexerStates = IndexerStates.INDEXER_FULL;
                }
                break;
            case INDEX_TO_A:
                green = false;
                robotHardware.indexerServo.setPosition(INDEXER_SERVO_POS_A);
                if (doesPosAHaveArtifact()) {
                    indexerPositions[0] = true;
//                    timer.reset();
//                    while(timer.milliseconds()<50);
                    indexerStates = IndexerStates.INDEX_TO_B;
                } else {
                    indexerStates = IndexerStates.INDEX_TO_A;
                }
                break;

            case INDEX_TO_B:
                robotHardware.indexerServo.setPosition(INDEXER_SERVO_POS_B);
                if (doesPosBHaveArtifact()) {
                    indexerPositions[1] = true;
//                    timer.reset();
//                    while(timer.milliseconds()<50);
                    indexerStates = IndexerStates.INDEX_TO_C;
                } else {
                    indexerStates = IndexerStates.INDEX_TO_B;
                }
                break;

            case INDEX_TO_C:
                robotHardware.indexerServo.setPosition(INDEXER_SERVO_POS_C);
                if (doesPosCHaveArtifact()) {
                    indexerPositions[2] = true;
//                    timer.reset();
//                    while(timer.milliseconds()<50);
                    indexerStates = IndexerStates.INDEXER_FULL;
                    green = true;
                } else {
                    indexerStates = IndexerStates.INDEX_TO_C;
                }
                break;

            case INDEXER_FULL:
                break;

        }
    }

    public void emptyIndexerArray() {
        indexerPositions[0] = false;
        indexerPositions[1] = false;
        indexerPositions[2] = false;
        indexerStates = IndexingClass.IndexerStates.INDEX_TO_A;
    }

    public void indexerCapacityStatus() {
        if (indexerStates == IndexerStates.INDEXER_FULL) {
            robotHardware.ledLight.setPosition(0.5);
        } else {
            robotHardware.ledLight.setPosition(0.278);
        }
    }



}
