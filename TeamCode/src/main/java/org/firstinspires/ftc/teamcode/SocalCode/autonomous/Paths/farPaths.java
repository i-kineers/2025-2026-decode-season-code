package org.firstinspires.ftc.teamcode.SocalCode.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class farPaths {

    // PATH NUMBER HAS TO BE REVERSED! FIRST INTAKE WOULD REFER TO THE ROWS OF BALLS CLOSEST TO THE BACKDROP BUT FOR FAR AUTO THE FIRST ROW OF BALLS WOULD BE THE THIRD!

    private Pose lastPose = null;

    // Heading angles are set to favor blue
    private double shootHeading = Math.toRadians(110);
    private double pickUpHeading = Math.toRadians(180);

    private double resetHeading = Math.toRadians(180);

    // Paths will be set to favor blue
    private Pose startPose = new Pose(52.969, 11.151);
    private Pose shootPose = new Pose(52.969, 11.151);
    private Pose homePose = new Pose(33.454,11.151);

    // All end poses for pickup in each 3 rows
    private Pose pickUpPose1 = new Pose(16, 80);
    private Pose pickUpPose2 = new Pose(10, 58.313);
    private Pose pickUpPose3 = new Pose(16, 35.71);

    // This is assuming the robot will always be going from the shooting to pick up
    private Pose pickupControl1 = new Pose(86.303, 86.424);
    private Pose pickupControl2 = new Pose(85.818, 63.272);
    private Pose pickupControl3 = new Pose(78.909, 37.818);

    // PathChain member variables, to be initialized in the constructor
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;

    public farPaths(Follower follower, boolean teamColor) {

        // Check if team color is red to reverse coordinates
        if (!teamColor) {
            shootHeading = reflect(shootHeading);
            pickUpHeading = reflect(pickUpHeading);
            resetHeading = reflect(resetHeading);
            startPose = reflect(startPose);
            shootPose = reflect(shootPose);
            pickUpPose1 = reflect(pickUpPose1);
            pickUpPose2 = reflect(pickUpPose2);
            pickUpPose3 = reflect(pickUpPose3);
            pickupControl1 = reflect(pickupControl1);
            pickupControl2 = reflect(pickupControl2);
            pickupControl3 = reflect(pickupControl3);
            homePose = reflect(homePose);
        }

        // From start to shooting
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(shootHeading)
                .build();

        // From shooting to intaking first rows of balls
        Path2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickupControl1, pickUpPose1))
                .addParametricCallback(0.3, () -> follower.setMaxPower(0.2))
                .setLinearHeadingInterpolation(shootHeading, pickUpHeading, 0.8)
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
                .addParametricCallback(0.45, () -> follower.setMaxPower(0.2))
                .setLinearHeadingInterpolation(shootHeading, pickUpHeading, 0.5)
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
                .addParametricCallback(0.25, () -> follower.setMaxPower(0.3))
                .setLinearHeadingInterpolation(shootHeading, pickUpHeading, 0.8)
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
