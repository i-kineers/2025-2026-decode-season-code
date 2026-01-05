package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.autonomous.Paths.autoAimPath;

public class AutoAimWithOdometry {

    private Follower follower;

    Pose blueGoalPose = new Pose(137.397, 142.394);
    Pose redGoalPose = new Pose(6.603, 142.394);
    Pose startPose = new Pose(22,120, Math.toRadians(135));
    Pose currentPose;

    public AutoAimWithOdometry(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
    }

    private double slopeCalculator(double currentX, double currentY, double targetHeadingX, double targetHeadingY) {
        return (currentY-targetHeadingY) / (currentX-targetHeadingX);
    }

    private double headingCalculator(double m1, double m2) {
        // Formula: tan(theta) = abs((m1 - m2) / (1 + (m1)(m2)))
        double tanTheta = Math.abs((m1-m2) / (1 + m1*m2));
        return Math.atan(tanTheta);
    }

    private void setHeading() {
        autoAimPath.getPath(follower, headingCalculator(0, slopeCalculator(follower.getPose().getX(), follower.getPose().getY(), blueGoalPose.getX(), blueGoalPose.getY())));
    }
}
