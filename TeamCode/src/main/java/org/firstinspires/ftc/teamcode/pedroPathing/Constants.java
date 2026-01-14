package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    static double ROBOT_MASS_IN_KG = 11.521;
    static double OTOS_LINEAR_SCALAR = 190.24164571428284;
    static double OTOS_ANGULAR_SCALAR = 1;
    static double X_VELOCITY = 64.65221014548474;
    static double Y_VELOCITY = 51.12054779773622;
    static double FORWARD_ZERO_POWER_ACCELERATION = -37.044193073376;
    static double LATERAL_ZERO_POWER_ACCELERATION = -64.87619107906065;
    static double CENTRIPETAL_SCALING = 0.0004;

    static SparkFunOTOS.Pose2D INITIAL_BOTPOSE = new SparkFunOTOS.Pose2D(2,1.75,Math.PI);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(ROBOT_MASS_IN_KG)
//            .useSecondaryTranslationalPIDF(true)
            .forwardZeroPowerAcceleration(FORWARD_ZERO_POWER_ACCELERATION)
            .lateralZeroPowerAcceleration(LATERAL_ZERO_POWER_ACCELERATION)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08,0,0.01,0.05))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.06,0.0238))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.5,0,0.00001,0.1,0))
            .centripetalScaling(CENTRIPETAL_SCALING);


    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
            .xVelocity(X_VELOCITY)
            .yVelocity(Y_VELOCITY);

    public static OTOSConstants otosConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .linearScalar(OTOS_LINEAR_SCALAR)
            .angularScalar(OTOS_ANGULAR_SCALAR)
            .offset(INITIAL_BOTPOSE);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 2, 0.5);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .OTOSLocalizer(otosConstants)
                .build();
    }
}
