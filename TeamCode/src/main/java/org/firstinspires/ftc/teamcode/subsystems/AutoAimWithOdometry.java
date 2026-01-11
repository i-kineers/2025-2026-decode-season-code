package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autonomous.Paths.autoAimPath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

public class AutoAimWithOdometry {
    private final Follower follower;
    private final PIDFController headingController;
    private boolean headingLock = false; // Toggle for heading lock

    // Field positions
    private final Pose blueGoalPose = new Pose(137.397, 142.394);
    private final Pose startPose = new Pose(22, 120, Math.toRadians(135));
    private double targetHeading;

    public AutoAimWithOdometry(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        headingController = new PIDFController(follower.constants.coefficientsHeadingPIDF);
    }

    // Toggle method for your TeleOp
    public void setHeadingLock(boolean active) {
        this.headingLock = active;
    }

    private double calculateTargetHeading() {
        Pose robot = follower.getPose();
        double dx = blueGoalPose.getX() - robot.getX();
        double dy = blueGoalPose.getY() - robot.getY();
        return MathFunctions.normalizeAngle(Math.atan2(dy, dx));
    }

    /**
     * Updated Loop Method
     * Pass gamepad inputs here to drive the robot while locking heading.
     */
    public void update(double left_stick_y, double left_stick_x, double right_stick_x) {
        follower.update();
        targetHeading = calculateTargetHeading();

        // Update PID with the smallest angle difference (from your image's logic)
        double error = MathFunctions.getSmallestAngleDifference(
                follower.getPose().getHeading(),
                targetHeading
        );
        headingController.updateError(error);

        if (headingLock) {
            // Use PID output for rotation instead of right stick
            follower.setTeleOpDrive(-left_stick_y, -left_stick_x, headingController.run(), false);
        } else {
            // Standard TeleOp driving
            follower.setTeleOpDrive(-left_stick_y, -left_stick_x, -right_stick_x, false);
        }
    }
}