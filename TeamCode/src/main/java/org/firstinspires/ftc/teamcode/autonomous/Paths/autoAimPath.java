package org.firstinspires.ftc.teamcode.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class autoAimPath {
    public static PathChain getPath(Follower follower, double calculatedHeading) {
        Pose currentPose = follower.getPose();

        // Safety check for null pose
        if (currentPose == null) {
            currentPose = new Pose(22, 120, 135);
        }

        return follower.pathBuilder()
                .addPath(new BezierLine(currentPose, currentPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), Math.toRadians(calculatedHeading))
                .build();
    }
}
