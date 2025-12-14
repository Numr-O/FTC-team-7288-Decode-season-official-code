package org.firstinspires.ftc.teamcode.mainCode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IndexerShootingAndIntake {

    ElapsedTime timer = new ElapsedTime();

    Servo ledLight;
    Servo indexerServo;
    Servo intakeServoLeft;
    Servo intakeServoRight;
    DcMotorEx shooterMotorTop;
    DcMotorEx shooterMotorBottom;
    DcMotor intakeMotor;
    ColorRangeSensor colorPosA;
    ColorRangeSensor colorPosB;
    ColorRangeSensor colorPosC;



    double SERVO_TRANSFER_POS_RIGHT = 0.63;
    double SERVO_TRANSFER_POS_LEFT = 0.4;

    double SERVO_TRAVEL_POS_RIGHT = 0.5;
    double SERVO_TRAVEL_POS_LEFT = 0.5;


    double INDEXER_SERVO_POS_A = 0.9;
    double INDEXER_SERVO_POS_B = 0.5;
    double INDEXER_SERVO_POS_C = 0.11;

    int TURRET_IDLE_SPEED = 1100;

    public int test = 1;

    public enum ShootingStates {
        SHOOTING_START,
        SHOOTING_A,
        SHOOTING_B,
        SHOOTING_C,
        INDEXER_EMPTY,
        WAITING_STATE_SHOOTER
    }
    public ShootingStates shootingState = ShootingStates.INDEXER_EMPTY;

    public enum IndexStates {
        INDEXER_EMPTY,
        INDEX_TO_POS_A,
        INDEX_TO_POS_B,
        INDEX_TO_POS_C,
        INDEXER_FULL,
        WAITING_STATE_INDEXER
    }
    IndexStates indexStates = IndexStates.INDEXER_EMPTY;

    public boolean areShooterMotorsAtSpeed;
    public boolean[] artifactAtIndexerPositions = {false,false,false};
    public boolean isInState = false;


    public IndexerShootingAndIntake(Object[] harwareList) {
        this.indexerServo = (Servo) harwareList[0];
        this.intakeServoLeft = (Servo) harwareList[1];
        this.intakeServoRight = (Servo) harwareList[2];
        this.shooterMotorTop = (DcMotorEx) harwareList[3];
        this.shooterMotorBottom = (DcMotorEx) harwareList[4];
        this.intakeMotor = (DcMotor) harwareList[5];
        this.colorPosA = (ColorRangeSensor) harwareList[6];
        this.colorPosB = (ColorRangeSensor) harwareList[7];
        this.colorPosC = (ColorRangeSensor) harwareList[8];
        this.ledLight = (Servo) harwareList[9];
    }

    private boolean doesPosAHaveBall () {
        return colorPosA.getDistance(DistanceUnit.MM) < 50;
    }
    private boolean doesPosBHaveBall () {
        return colorPosB.getDistance(DistanceUnit.MM) < 50;
    }
    private boolean doesPosCHaveBall () {
        return colorPosC.getDistance(DistanceUnit.MM) < 50;
    }


    public void ShootingOrIndexingArtifacts(int shooterVelocity, boolean isTriggerPressed) {
        if (isTriggerPressed) {
            indexStates = IndexStates.WAITING_STATE_INDEXER;
            intakeMotor.setPower(-1);
            shootBalls(shooterVelocity);
        } else {
            shootingState = ShootingStates.WAITING_STATE_SHOOTER;
            shooterMotorTop.setVelocity(-TURRET_IDLE_SPEED);
            shooterMotorBottom.setVelocity(TURRET_IDLE_SPEED);
            indexBalls();
        }
    }

    public void shootBalls (int shooterVelocity) {
        boolean isTopMotorAtSpeed = shooterMotorTop.getVelocity() >= -shooterVelocity - 30 && shooterMotorTop.getVelocity() <= -shooterVelocity + 30;
        boolean isBottomMotorAtSpeed = shooterMotorBottom.getVelocity() >= shooterVelocity - 30 && shooterMotorBottom.getVelocity() <= shooterVelocity + 30;

        areShooterMotorsAtSpeed = isTopMotorAtSpeed && isBottomMotorAtSpeed;
        switch (shootingState) {
            case WAITING_STATE_SHOOTER:
                timer.reset();
                while (timer.milliseconds() < 10);
                shootingState = ShootingStates.INDEXER_EMPTY;
                break;
            case INDEXER_EMPTY:
                if (artifactAtIndexerPositions[2]) {
                    shootingState = ShootingStates.SHOOTING_C;
                    break;
                } else if (artifactAtIndexerPositions[1]) {
                    shootingState = ShootingStates.SHOOTING_B;
                    break;
                } else if (artifactAtIndexerPositions[0]) {
                    shootingState = ShootingStates.SHOOTING_A;
                    break;
                } else {
                    break;
                }
            case SHOOTING_C:
                shooterMotorTop.setVelocity(-shooterVelocity);
                shooterMotorBottom.setVelocity(shooterVelocity);
                timer.reset();
                while (timer.milliseconds() < 300);
                if (areShooterMotorsAtSpeed) {
                    intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
                    intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);
                    timer.reset();
                    while (timer.milliseconds() < 450);
                    artifactAtIndexerPositions[2] = false;
                    intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                    intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);
                    shootingState = ShootingStates.INDEXER_EMPTY;
                    break;
                } else {
                    shootingState = ShootingStates.SHOOTING_C;
                    break;
                }

            case SHOOTING_B:
                shooterMotorTop.setVelocity(-shooterVelocity);
                shooterMotorBottom.setVelocity(shooterVelocity);
                indexerServo.setPosition(INDEXER_SERVO_POS_B);
                timer.reset();
                while (timer.milliseconds() < 300);
                if (areShooterMotorsAtSpeed) {
                    intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
                    intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);
                    timer.reset();
                    while (timer.milliseconds() < 450);
                    artifactAtIndexerPositions[1] = false;
                    intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                    intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);
                    shootingState = ShootingStates.INDEXER_EMPTY;
                    break;
                } else {
                    shootingState = ShootingStates.SHOOTING_B;
                    break;
                }
            case SHOOTING_A:
                shooterMotorTop.setVelocity(-shooterVelocity);
                shooterMotorBottom.setVelocity(shooterVelocity);
                indexerServo.setPosition(INDEXER_SERVO_POS_A);
                timer.reset();
                while (timer.milliseconds() < 300);
                if (areShooterMotorsAtSpeed) {
                    intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
                    intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);
                    timer.reset();
                    while (timer.milliseconds() < 450);
                    artifactAtIndexerPositions[0] = false;
                    intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                    intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);
                    shootingState = ShootingStates.INDEXER_EMPTY;
                    break;
                } else {
                    shootingState = ShootingStates.SHOOTING_A;
                    break;
                }
        }
    }

    public ShootingStates getShooterState () {
        return shootingState;
    }



    public void indexBalls() {
        switch (indexStates) {
            case WAITING_STATE_INDEXER:
                timer.reset();
                while (timer.milliseconds() < 10);
                indexStates = IndexStates.INDEXER_EMPTY;
                break;
            case INDEXER_EMPTY:

                if (artifactAtIndexerPositions[0] == false) {
                    ledLight.setPosition(0.277);
                    indexStates = IndexStates.INDEX_TO_POS_A;
                    break;
                } else if (artifactAtIndexerPositions[1] == false) {
                    ledLight.setPosition(0.333);
                    indexStates = IndexStates.INDEX_TO_POS_B;
                    break;
                } else if (artifactAtIndexerPositions[2] == false) {
                    ledLight.setPosition(0.388);
                    indexStates = IndexStates.INDEX_TO_POS_C;
                    break;
                } else {
                    ledLight.setPosition(0.500);
                    indexStates = IndexStates.INDEXER_FULL;
                    break;
                }
            case INDEX_TO_POS_A:
                indexerServo.setPosition(INDEXER_SERVO_POS_A);
                if (doesPosAHaveBall()) {
                    timer.reset();
                    while(timer.milliseconds() < 50);
                    artifactAtIndexerPositions[0] = true;
                    indexStates = IndexStates.INDEXER_EMPTY;
                    break;
                } else {
                    indexStates = IndexStates.INDEX_TO_POS_A;
                    break;
                }
            case INDEX_TO_POS_B:
                indexerServo.setPosition(INDEXER_SERVO_POS_B);
                if (doesPosBHaveBall()) {
                    timer.reset();
                    while(timer.milliseconds() < 50);
                    artifactAtIndexerPositions[1] = true;
                    indexStates = IndexStates.INDEXER_EMPTY;
                    break;
                } else {
                    indexStates = IndexStates.INDEX_TO_POS_B;
                    break;
                }
            case INDEX_TO_POS_C:
                indexerServo.setPosition(INDEXER_SERVO_POS_C);
                if (doesPosCHaveBall()) {
                    timer.reset();
                    while(timer.milliseconds() < 50);
                    artifactAtIndexerPositions[2] = true;
                    indexStates = IndexStates.INDEXER_EMPTY;
                    break;
                } else {
                    indexStates = IndexStates.INDEX_TO_POS_C;
                    break;
                }
            case INDEXER_FULL:
                if (artifactAtIndexerPositions[0] == false || artifactAtIndexerPositions[1] == false || artifactAtIndexerPositions[2] == false) {
                    indexStates = IndexStates.INDEXER_EMPTY;
                    break;
                } else {
                    indexStates = IndexStates.INDEXER_FULL;
                }
        }
    }

    public IndexStates getIndexingState () {
        return indexStates;
    }


    public void autoShooting(double shooterVelocity) {

        boolean isTopMotorAtSpeed = shooterMotorTop.getVelocity() >= -shooterVelocity - 30 && shooterMotorTop.getVelocity() <= -shooterVelocity + 30;
        boolean isBottomMotorAtSpeed = shooterMotorBottom.getVelocity() >= shooterVelocity - 30 && shooterMotorBottom.getVelocity() <= shooterVelocity + 30;

        areShooterMotorsAtSpeed = isTopMotorAtSpeed && isBottomMotorAtSpeed;

        intakeMotor.setPower(-1);

        switch (test) {
            case 1:
                shooterMotorTop.setVelocity(-shooterVelocity);
                shooterMotorBottom.setVelocity(shooterVelocity);

                intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);

                indexerServo.setPosition(INDEXER_SERVO_POS_C);
                if (areShooterMotorsAtSpeed) {
                    isInState = true;
                    timer.reset();
                    while (timer.milliseconds() < 1000) {
                        continue;
                    }
                    intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
                    intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);
                    timer.reset();
                    while (timer.milliseconds() < 1000) {
                        continue;
                    }
                    intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                    intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);
                    test = 2;
                    break;
                } else {
                    break;
                }
            case 2:
                shooterMotorTop.setVelocity(-shooterVelocity);
                shooterMotorBottom.setVelocity(shooterVelocity);
                if (areShooterMotorsAtSpeed) {
                    indexerServo.setPosition(INDEXER_SERVO_POS_B);
                    timer.reset();
                    while (timer.milliseconds() < 1000) {
                        continue;
                    }
                    intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
                    intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);
                    timer.reset();
                    while (timer.milliseconds() < 1000) {
                        continue;
                    }
                    intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                    intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);
                    test = 3;
                    break;
                } else {
                    break;
                }
            case 3:
                shooterMotorTop.setVelocity(-shooterVelocity);
                shooterMotorBottom.setVelocity(shooterVelocity);
                if (areShooterMotorsAtSpeed) {
                    indexerServo.setPosition(INDEXER_SERVO_POS_A);
                    timer.reset();
                    while (timer.milliseconds() < 1000) {
                        continue;
                    }
                    intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
                    intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);
                    timer.reset();
                    while (timer.milliseconds() < 1000) {
                        continue;
                    }
                    intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                    intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);
                    test = 4;
                    break;
                } else {
                    break;
                }
            case 4:
                break;
        }
    }

}
