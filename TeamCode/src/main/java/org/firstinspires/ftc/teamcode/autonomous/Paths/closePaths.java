package org.firstinspires.ftc.teamcode.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class closePaths {

    private Pose lastPose = null;

    // Heading angles are set to favor blue
    private double shootHeading = Math.toRadians(135);
    private double pickUpHeading = Math.toRadians(180);

    private double resetHeading = Math.toRadians(90);

    // Paths will be set to favor blue
    private Pose startPose = new Pose(22, 120);
    private Pose shootPose = new Pose(64, 80);
    private Pose homePose = new Pose(38.711, 32.205);

    // All end poses for pickup in each 3 rows
    private Pose pickUpPose1 = new Pose(19, 84.097);
    private Pose pickUpPose2 = new Pose(18.855, 59.569);
    private Pose pickUpPose3 = new Pose(19.022, 35.71);

    // This is assuming the robot will always be going from the shooting to pick up
    private Pose pickupControl1 = new Pose(56.899, 84.598);
    private Pose pickupControl2 = new Pose(65.242, 59.569);
    private Pose pickupControl3 = new Pose(72.083, 32.037);

    // PathChain member variables, to be initialized in the constructor
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;

    public closePaths(Follower follower, int pathSelection, boolean teamColor) {

        // Check if team color is red to reverse coordinates
        if (!teamColor) {
            shootHeading = reflect(shootHeading);
            pickUpHeading = reflect(pickUpHeading);
            startPose = reflect(startPose);
            shootPose = reflect(shootPose);
            pickUpPose1 = reflect(pickUpPose1);
            pickUpPose2 = reflect(pickUpPose2);
            pickUpPose3 = reflect(pickUpPose3);
            pickupControl1 = reflect(pickupControl1);
            pickupControl2 = reflect(pickupControl2);
            pickupControl3 = reflect(pickupControl3);
        }

        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(shootHeading)
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickUpPose1))
                .setLinearHeadingInterpolation(shootHeading, pickUpHeading)
                .build();

        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose1, shootPose))
                .setLinearHeadingInterpolation(pickUpHeading, pickUpHeading)
                .build();

        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, homePose))
                .setLinearHeadingInterpolation(pickUpHeading, pickUpHeading)
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
