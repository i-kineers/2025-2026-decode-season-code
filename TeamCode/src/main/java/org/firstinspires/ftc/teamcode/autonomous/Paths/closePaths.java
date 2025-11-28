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
    private Pose startPose = new Pose(21.192, 121.4186);
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
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

    public closePaths(Follower follower, boolean teamColor) {

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

        // From start to shooting
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(shootHeading)
                .build();

        // From shooting to intaking first rows of balls
        Path2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickupControl1, pickUpPose1))
                .setLinearHeadingInterpolation(shootHeading, pickUpHeading)
                .build();

        // From intaking first row of balls to shooting
        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose1, shootPose))
                .setLinearHeadingInterpolation(pickUpHeading, shootHeading)
                .build();

        // Path 4 and 5 would be when the user selects 2 rows of balls to intake
        // Move from shooting position to intake 2nd row of balls
        Path4 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickupControl2, pickUpPose2))
                .setLinearHeadingInterpolation(shootHeading, pickUpHeading)
                .build();
        // Move from intake 2nd row of balls to shooting position
        Path5 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose2, shootPose))
                .setLinearHeadingInterpolation(pickUpHeading, shootHeading)
                .build();

        // Path 6 and 7 would be when the user selects all(3) rows of balls to intake
        // Move from shooting position to intake 3rd row of balls
        Path6 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickupControl3, pickUpPose3))
                .setLinearHeadingInterpolation(shootHeading, pickUpHeading)
                .build();
        // Move from intake 3rd row of balls to shooting position
        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose3, shootPose))
                .setLinearHeadingInterpolation(pickUpHeading, shootHeading)
                .build();

        // Final path that exits the shooting line
        Path8 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, homePose))
                .setLinearHeadingInterpolation(shootHeading, resetHeading)
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
