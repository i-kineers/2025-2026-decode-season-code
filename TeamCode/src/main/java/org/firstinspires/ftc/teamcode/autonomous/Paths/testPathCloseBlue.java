package org.firstinspires.ftc.teamcode.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class testPathCloseBlue {

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;

    public testPathCloseBlue(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(22.000, 120.000), new Pose(48,95))
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .setGlobalDeceleration(0.8)
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                    new BezierCurve(
                            new Pose(48.000, 95.000),
                            new Pose(64.059, 80.833),
                            new Pose(16.000, 80)
                    )
                )
                .addParametricCallback(0.25, () -> follower.setMaxPower(0.25))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180), 0.8)
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16, 80), new Pose(48,95))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135), 0.8)
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48,95), new Pose(58, 112))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180), 0.8)
                .build();
    }
}
