package org.firstinspires.ftc.teamcode.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class Paths {
    public static Path scorePreload;
    public static PathChain grabPickup1;
    public static Pose startPose = new Pose(22, 120, Math.toRadians(135));
    public static Pose scorePose = new Pose(47, 97, Math.toRadians(135));
    public static Pose endPose = new Pose(72, 22, Math.toRadians(180));

    public static void buildPaths(Follower follower) {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading(), 0.8)
                .build();
    }
}