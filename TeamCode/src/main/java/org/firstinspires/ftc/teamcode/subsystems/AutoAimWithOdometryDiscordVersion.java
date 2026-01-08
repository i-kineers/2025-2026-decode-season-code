//package org.firstinspires.ftc.teamcode.subsystems;
//
//public class AutoAimWithOdometryDiscordVersion {
//    public double getHeadingToGoal() {
//        // 1. Get the current position of the robot from the localizer/follower
//        Pose robotPose = follower.getPose();
//
//        // 2. Predictive Aiming: Adjust for the robot's movement
//        // Multiplies velocity by a time constant (0.65s) to predict future position
//        if (useVelocityCompensation) {
//            robotPose.add(follower.getVelocity().returnMultiplied(0.65).toPose());
//        }
//
//        // 3. Select the target goal based on the alliance color
//        Pose goalPos = Parameters.RED_SHOOTER_GOAL;
//
//        if (side == AllianceSides.BLUE) {
//            goalPos = Parameters.BLUE_SHOOTER_GOAL;
//        }
//
//        // 4. Calculate the angle between the (predicted) robot position and the goal
//        // Adding 180 degrees (Math.PI) because the shooter is on the back of the robot
//        return (Math.atan2(goalPos.getY() - robotPose.getY(), goalPos.getX() - robotPose.getX()) + Math.toRadians(180));
//    }
//}
