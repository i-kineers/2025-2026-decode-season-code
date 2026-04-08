package org.firstinspires.ftc.teamcode.SocalCode.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.opencv.core.Mat;

public class farAutoPaths {

    // Default Blue Headings (Converted from your static SHOOT_HEADING constants)
    private double startHeading = Math.toRadians(180);
    private double shootHeading = Math.toRadians(203); // SHOOT_HEADING
    private double shootHeading2 = Math.toRadians(203); // shoot_heading_2
    private double cornerHeading1 = Math.toRadians(188.0); // CORNER_HEADING_1
    private double cornerHeading2 = Math.toRadians(172.0); // CORNER_HEADING_2
    private double intake3Heading = Math.toRadians(180); // intake3_h
    private double parkHeading = Math.toRadians(180); // pFar_end_h

    // Default Blue Poses
    private Pose startPose = new Pose(64.000, 8.000);
    private Pose shootPose = new Pose(60.000, 17.000);
    private Pose cornerPose = new Pose(12.25, 9.0); // CORNER_X/Y_PADDING
    private Pose intake3Pose = new Pose(12.000, 38.5); // intake3_pose (36 + 2.5)
    private Pose parkPose = new Pose(48.000, 20.000); // pFar_end

    // Control Points
    private Pose cornerControl1 = new Pose(52.000, 6.700); // corner_c1
    private Pose cornerControl2 = new Pose(17.000, 17.000); // corner_c2
    private Pose cornerControl3 = new Pose(8.000, 23.000); // corner_c3
    private Pose cornerControl4 = new Pose(12.25, 9.0); // corner_c4
    private Pose intake3Control1 = new Pose(60.000, 42.000); // intake3_c1

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, PathPark;

    public farAutoPaths(Follower follower, boolean isBlueSide, boolean useSpike) {

        if (!isBlueSide) {
            // Reflect Headings
            startHeading = Math.toRadians(180);
            shootHeading = Math.toRadians(157);
            shootHeading2 = Math.toRadians(157);
            cornerHeading1 = Math.toRadians(172);
            cornerHeading2 = Math.toRadians(180);
            intake3Heading = Math.toRadians(180);
            parkHeading = Math.toRadians(180);

            // Reflect Poses
            startPose = reflect(startPose);
            shootPose = reflect(shootPose);
            cornerPose = reflect(cornerPose);
            intake3Pose = reflect(intake3Pose);
            parkPose = reflect(parkPose);

            // Reflect Controls
            cornerControl1 = reflect(cornerControl1);
            cornerControl2 = reflect(cornerControl2);
            cornerControl3 = reflect(cornerControl3);
            cornerControl4 = reflect(cornerControl4);
            intake3Control1 = reflect(intake3Control1);
        }

        // Path 1: Start to Shoot
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startHeading, shootHeading)
                .build();

        // Path 2: Shoot to Corner (Intake Index 4)
        Path2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, cornerControl2, cornerControl3, cornerPose))
                .setLinearHeadingInterpolation(shootHeading, cornerHeading1)
                .build();

        // Path 3: Corner to Shoot
        Path3 = follower.pathBuilder()
                .addPath(new BezierCurve(cornerPose, cornerControl4, shootPose))
                .setLinearHeadingInterpolation(cornerHeading1, shootHeading2)
                .build();

        // Path 4: Shoot to Corner (Cycle 2)
        Path4 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, cornerControl1, cornerPose))
                .setLinearHeadingInterpolation(shootHeading2, cornerHeading2)
                .build();

        // Path 5: Corner to Shoot (Cycle 2)
        Path5 = follower.pathBuilder()
                .addPath(new BezierLine(cornerPose, shootPose))
                .setLinearHeadingInterpolation(cornerHeading2, shootHeading2)
                .build();

        // Path 6: Shoot to Corner (Cycle 3)
        Path6 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, cornerControl1, cornerPose))
                .setLinearHeadingInterpolation(shootHeading2, cornerHeading2)
                .build();

        // Path 7: Corner to Shoot (Cycle 3)
        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(cornerPose, shootPose))
                .setLinearHeadingInterpolation(cornerHeading2, shootHeading2)
                .build();

        if (useSpike) {
            // Path 8: Shoot to Intake 3 (Far Spike)
            Path8 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPose, intake3Control1, intake3Pose))
                    .setLinearHeadingInterpolation(shootHeading2, intake3Heading)
                    .build();
            // Path 9: Intake 3 to Shoot
            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(intake3Pose, shootPose))
                    .setLinearHeadingInterpolation(intake3Heading, shootHeading2)
                    .build();
        } else {
            // Path 8: Shoot to Corner (Alternative Cycle 4)
            Path8 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPose, cornerControl1, cornerPose))
                    .setLinearHeadingInterpolation(shootHeading2, cornerHeading2)
                    .build();
            // Path 9: Corner to Shoot
            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(cornerPose, shootPose))
                    .setLinearHeadingInterpolation(cornerHeading2, shootHeading2)
                    .build();
        }

        // Final Park Path
        PathPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootHeading2, parkHeading)
                .build();
    }

    private Pose reflect(Pose pose) {
        return new Pose(144 - pose.getX(), pose.getY(), 0);
    }

    private double reflect(double angleInRadians) {
        double reflected = Math.PI - angleInRadians;
        while (reflected <= -Math.PI) reflected += 2 * Math.PI;
        while (reflected > Math.PI) reflected -= 2 * Math.PI;
        return reflected;
    }
}