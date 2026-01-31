package org.firstinspires.ftc.teamcode.SocalCode.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class testPathFarRed {
    public PathChain Path1;

    public testPathFarRed(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(91.031, 11.151), new Pose(110.546, 11.151))
                )
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(90))
                .build();
    }
}
