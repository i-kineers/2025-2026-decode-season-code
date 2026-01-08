package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.autonomous.Paths.autoAimPath;

public class AutoAimWithOdometry {

    private double alignHeadingAngle;

    private Follower follower;

    Pose blueGoalPose = new Pose(137.397, 142.394);
    Pose startPose = new Pose(22,120, Math.toRadians(135));
    Pose currentPose;

    public AutoAimWithOdometry(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
    }

    private double headingCalculator() {
        // m1 and m2 representing 2 different lines slopes
        // Formula: tan(theta) = abs((m1 - m2) / (1 + (m1)(m2)))
        // Answer will always be acute angle, next formula shown is the correct modified one

        // Formula: tan(theta) = (m1 - m2) / (1 + (m1)(m2))
        // This COULD result in a negative number which if it does, then take 180 - result

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        // Instead of slopes, we use the "Vector" between the two points
        double m1 = (blueGoalPose.getY() - robotY) / (blueGoalPose.getX() - robotX);

        double tanTheta = (m1-0) / (1 + m1*0);
        return Math.atan(tanTheta);
    }

    private double setHeading() {
        double alignAngle = headingCalculator();

        if (alignAngle < 0) {
            alignHeadingAngle = (180 - alignAngle);
        } else if (alignAngle > 0) {
            alignHeadingAngle = alignAngle;
        } else {
            alignHeadingAngle = alignAngle;
        }

        return alignAngle;
    }

    private void runAlignment() {
        // Use PID for this. Should be similar to camera alignment.
        PathChain path = autoAimPath.getPath(follower, setHeading());
    }

}
