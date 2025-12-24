package org.firstinspires.ftc.teamcode.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class teleopPath {

    // Default target pose for the teleop path
    private static final Pose DEFAULT_TARGET = new Pose(48, 95, Math.toRadians(135));

    /**
     * Generates a path from the follower's current pose to the default target pose.
     * @param follower The follower instance to get the current pose from.
     * @return A PathChain from current pose to default target.
     */
    public static PathChain getPath(Follower follower) {
        return getPath(follower, DEFAULT_TARGET);
    }

    /**
     * Generates a path from the follower's current pose to a specified target pose.
     * @param follower The follower instance to get the current pose from.
     * @param targetPose The target pose to drive to.
     * @return A PathChain from current pose to target pose.
     */
    public static PathChain getPath(Follower follower, Pose targetPose) {
        Pose currentPose = follower.getPose();
        
        // Safety check for null pose
        if (currentPose == null) {
            currentPose = new Pose(0, 0, 0); 
        }

        return follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .build();
    }
}
