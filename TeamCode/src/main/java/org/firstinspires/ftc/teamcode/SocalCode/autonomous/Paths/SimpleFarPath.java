package org.firstinspires.ftc.teamcode.SocalCode.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


public class SimpleFarPath {
    public PathChain Path1;
    public PathChain Path2;

    public SimpleFarPath(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.833, 5.799),

                                new Pose(64.030, 15.123)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(210))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(64.030, 15.123),

                                new Pose(44.948, 15.123)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180))

                .build();
    }
}

