package org.firstinspires.ftc.teamcode.CodePriorILT.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class testPathFarRed {
    public PathChain Path1;
    public PathChain Path2;

    public testPathFarRed(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(88.727, 7.516),

                                new Pose(89.211, 14.079)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(89.211, 14.079),

                                new Pose(107.961, 14.079)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(90))

                .build();
    }
}