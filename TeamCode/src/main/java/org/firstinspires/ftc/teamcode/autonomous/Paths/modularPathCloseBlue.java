package org.firstinspires.ftc.teamcode.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class modularPathCloseBlue {

    public PathChain FirstShot;
    public PathChain FirstPickUp;
    public PathChain SecondShot;
    public PathChain SecondPickUp;
    public PathChain ThirdShot;
    public PathChain ThirdPickUp;
    public PathChain FourthShot;
    public PathChain GoHome;

    public modularPathCloseBlue(Follower follower) {
        FirstShot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(22.000, 120.000), new Pose(59.068, 84.097))
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        FirstPickUp = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.068, 84.097), new Pose(19.189, 84.097))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        SecondShot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.189, 84.097), new Pose(72.083, 72.751))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        SecondPickUp = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(72.083, 72.751),
                                new Pose(77.757, 57.567),
                                new Pose(19.189, 59.402)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        ThirdShot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.189, 59.402), new Pose(72.417, 72.584))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        ThirdPickUp = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(72.417, 72.584),
                                new Pose(70.749, 32.871),
                                new Pose(18.855, 35.374)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        FourthShot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.855, 35.374), new Pose(71.917, 72.250))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        GoHome = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(71.917, 72.250), new Pose(49.224, 17.854))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();
    }
}
