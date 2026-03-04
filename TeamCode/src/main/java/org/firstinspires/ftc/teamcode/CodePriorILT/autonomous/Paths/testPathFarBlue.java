package org.firstinspires.ftc.teamcode.CodePriorILT.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class testPathFarBlue {
    public PathChain Path1;
    public PathChain Path2;

    public testPathFarBlue(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.273, 7.516),

                                new Pose(54.789, 14.079)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.789, 14.079),

                                new Pose(36.039, 14.079)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(90))

                .build();
    }
}