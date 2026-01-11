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
    private final Pose blueGoalPose = new Pose(6.719, 142.394);
    private final Pose startPose = new Pose(22, 120, Math.toRadians(135));
    private double targetHeading;
    private double lastTurn;

    public AutoAimWithOdometry(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        headingController = new PIDFController(follower.constants.coefficientsHeadingPIDF);
    }

    // Toggle method for your TeleOp
    public void setHeadingLock(boolean active) {
        if (active && !headingLock) {
            headingController.reset(); // VERY IMPORTANT
        }
        headingLock = active;
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

        // Always calculate target heading toward the backdrop
        targetHeading = calculateTargetHeading(); // points at blueGoalPose

        // --- Calculate error using shortest rotation path ---
        double currentHeading = follower.getPose().getHeading();
        double error = targetHeading - currentHeading;

        // Normalize to [-π, π]
        if (error > Math.PI) {
            error -= 2 * Math.PI;
        } else if (error < -Math.PI) {
            error += 2 * Math.PI;
        }

        // Deadband for small errors
        if (Math.abs(error) < Math.toRadians(1.5)) {
            error = 0;
            lastTurn = 0;
        } else {
            headingController.updateError(error);
            lastTurn = headingController.run();

            // Ramp down turn power near target
            double maxTurn = 0.4;
            if (Math.abs(error) < Math.toRadians(10)) {
                maxTurn = 0.2;
            }

            lastTurn = MathFunctions.clamp(lastTurn, -maxTurn, maxTurn);
        }

        // Apply drive
        if (headingLock) {
            // Now heading lock actually points at the target
            follower.setTeleOpDrive(-left_stick_y, -left_stick_x, lastTurn, false);
        } else {
            follower.setTeleOpDrive(-left_stick_y, -left_stick_x, -right_stick_x, false);
        }
    }


    public boolean isHeadingLockEnabled() {
        return headingLock;
    }

    public double getTargetHeadingDeg() {
        return Math.toDegrees(targetHeading);
    }

    public double getCurrentHeadingDeg() {
        return Math.toDegrees(follower.getPose().getHeading());
    }

    public double getHeadingErrorDeg() {
        double error = targetHeading - follower.getPose().getHeading();
        // Normalize to [-π, π]
        if (error > Math.PI) {
            error -= 2 * Math.PI;
        } else if (error < -Math.PI) {
            error += 2 * Math.PI;
        }
        return Math.toDegrees(error);
    }

    public double getTurnPower() {
        return lastTurn;
    }
}
