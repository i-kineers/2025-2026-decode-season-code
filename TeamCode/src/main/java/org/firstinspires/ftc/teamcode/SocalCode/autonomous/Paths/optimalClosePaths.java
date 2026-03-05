package org.firstinspires.ftc.teamcode.SocalCode.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class optimalClosePaths {

    // Heading angles are set to favor blue
    private double startHeading = Math.toRadians(135);
    private double gatePickUpHeading = Math.toRadians(135);
    private double shootHeading = Math.toRadians(225);
    private double pickUpHeading = Math.toRadians(180);
    private double angledPickUpHeading = Math.toRadians(225);
    private double resetHeading = Math.toRadians(180);

    // Paths will be set to favor blue
    private Pose startPose = new Pose(19.5, 122.6);
    private Pose shootPose = new Pose(55.315,88.219);
    private Pose homePose = new Pose(58, 112);
    private Pose gatePose = new Pose(16, 70);


    // All end poses for pickup in each 3 rows
    private Pose spike1 = new Pose(13.34, 84.572);
    private Pose spike2 = new Pose(8, 59.345);
    private Pose midSpike2 = new Pose(55.315, 61.200);
    private Pose gatePickupPose = new Pose(7.353, 59.535);

    // This is assuming the robot will always be going from the shooting to pick up
    private Pose spike2Control = new Pose(62.453, 62.453);
    private Pose gateControl = new Pose(42.078, 64.881);
    private Pose spike1Control = new Pose(38.931, 83.452);
    private Pose returnPose2 = new Pose(58.253, 60.628);
    private Pose pickupControl3 = new Pose(81.87, 31.63);

    // PathChain member variables, to be initialized in the constructor
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

    public optimalClosePaths(Follower follower, boolean teamColor) {

        // Check if team color is red to reverse coordinates
        if (!teamColor) {
            shootHeading = reflect(shootHeading);
            pickUpHeading = reflect(pickUpHeading);
            resetHeading = reflect(resetHeading);
            startPose = reflect(startPose);
            shootPose = reflect(shootPose);
            gatePose = reflect(gatePose);
            spike2 = reflect(spike2);
            spike1 = reflect(spike1);
            returnPose2 = reflect(returnPose2);
            spike2Control = reflect(spike2Control);
            gateControl = reflect(gateControl);
            pickupControl3 = reflect(pickupControl3);
            gateControl = reflect(gateControl);
            homePose = reflect(homePose);
            gatePickupPose = reflect(gatePickupPose);
            spike1Control = reflect(spike1Control);
        }


        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,

                                shootPose
                        )
                ).setLinearHeadingInterpolation(startHeading, shootHeading)

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPose,
                                midSpike2
                        )
                ).setLinearHeadingInterpolation(shootHeading, 0, 0.1)
                .addPath(
                        new BezierLine(
                                midSpike2,
                                spike2
                        )
                ).setConstantHeadingInterpolation(0)
                .addParametricCallback(0.2, () -> follower.setMaxPower(0.5))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                spike2,
                                spike2Control,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(0, shootHeading)

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPose,
                                gateControl,
                                gatePickupPose
                        )
                ).setLinearHeadingInterpolation(shootHeading, gatePickUpHeading)

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                gatePickupPose,
                                gateControl,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(gatePickUpHeading, shootHeading)

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPose,
                                gateControl,
                                gatePickupPose
                        )
                ).setLinearHeadingInterpolation(shootHeading, gatePickUpHeading)

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                gatePickupPose,
                                gateControl,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(gatePickUpHeading, shootHeading)

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPose,
                                spike1Control,
                                spike1
                        )
                ).setLinearHeadingInterpolation(shootHeading, pickUpHeading)

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                spike1,

                                shootPose
                        )
                ).setLinearHeadingInterpolation(pickUpHeading, shootHeading)

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPose,

                                homePose
                        )
                ).setLinearHeadingInterpolation(shootHeading, pickUpHeading)

                .build();
    }

    private Pose reflect(Pose pose) {
        // We pass 0 for heading here because the heading is reflected separately.
        return new Pose(144 - pose.getX(), pose.getY(), 0);
    }

    private double reflect(double angleInRadians) {
        return Math.PI - angleInRadians;
    }

}
