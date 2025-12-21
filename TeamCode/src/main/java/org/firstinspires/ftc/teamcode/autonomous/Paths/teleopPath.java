package org.firstinspires.ftc.teamcode.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class teleopPath {

    public PathChain Path1;

    public teleopPath(Follower follower, double x, double y, double targetPoseX, double targetPoseY) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x, y), new Pose(targetPoseX, targetPoseY))
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .setGlobalDeceleration(0.8)
                .build();
    }
}
