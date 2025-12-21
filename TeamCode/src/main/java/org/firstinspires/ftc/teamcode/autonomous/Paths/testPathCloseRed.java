package org.firstinspires.ftc.teamcode.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class testPathCloseRed {

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;

    public testPathCloseRed(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(122, 120), new Pose(96, 95.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(96, 95.000),
                                new Pose(79.941, 80.833),
                                new Pose(128, 80)
                        )
                )
                .addParametricCallback(0.25, () -> follower.setMaxPower(0.25))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0), 0.8)
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(128, 84.097), new Pose(96, 95.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45), 0.8)
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96, 95.000), new Pose(86, 112)) // 21, 14
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0), 0.8)
                .build();
    }
}
