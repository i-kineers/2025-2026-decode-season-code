package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(9); // Mass of meet 1 bot: 8.845051
//        .forwardZeroPowerAcceleration(-38.9)
//        .lateralZeroPowerAcceleration(-47.386);
//        .translationalPIDFCoefficients(new PIDFCoefficients(0.05,0,0.005,0.01))
//        .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.05,0.01))
//        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,0.00001,0.6, 0.01))
//        .centripetalScaling(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .xVelocity(63.77)
        .yVelocity(57)
        .rightFrontMotorName("frmotor")
        .rightRearMotorName("brmotor")
        .leftRearMotorName("blmotor")
        .leftFrontMotorName("flmotor")
        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
        .forwardPodY(5.690472) // New value for meet 1
        .strafePodX(-0.9308268) // New value for meet 1
        .distanceUnit(DistanceUnit.INCH)
        .hardwareMapName("pinpoint")
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
