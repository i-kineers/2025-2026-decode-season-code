package org.firstinspires.ftc.teamcode.autonomous.Paths;

import com.pedropathing.follower.Follower;
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
                        new BezierLine(new Pose(22.000, 120.000), new Pose(59.068, 84.097))
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.068, 84.097), new Pose(19.189, 84.097))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.189, 84.097), new Pose(59.068, 84.097))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.068, 84.097), new Pose(49.224, 17.854))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();
    }
}
