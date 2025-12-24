package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autonomous.Paths.teleopPath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class TeleOpPathingManager {
    private Follower follower;
    private boolean automatedDrive = false;

    private static Pose targetPose = new Pose(48, 95, Math.toRadians(135));

    public TeleOpPathingManager(HardwareMap hardwareMap, Pose startingPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.startTeleopDrive();
    }

    public void update() {
        follower.update();
    }

    /**
     * Handles TeleOp driving and automated path triggering.
     * @param forward Forward movement (e.g. gamepad.left_stick_y)
     * @param strafe Strafe movement (e.g. gamepad.left_stick_x)
     * @param turn Turn movement (e.g. -gamepad.right_stick_x)
     * @param triggerPath Boolean to trigger the automated path (e.g. gamepad.a)
     */
    public void drive(double forward, double strafe, double turn, boolean triggerPath) {
        boolean manualInput = Math.abs(forward) > 0.1 || Math.abs(strafe) > 0.1 || Math.abs(turn) > 0.1;

        // Trigger Pathing
        if (triggerPath && !automatedDrive) {
            PathChain path = teleopPath.getPath(follower, targetPose);
            follower.followPath(path);
            automatedDrive = true;
        }

        // Stop Pathing if done or manual override
        if (automatedDrive) {
            if (!follower.isBusy() || manualInput) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }
        }

        // Manual Drive
        if (!automatedDrive) {
            follower.setTeleOpDrive(forward, strafe, turn, false);
        }
    }

    public void setTargetPose() {
        targetPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(follower.getPose().getHeading()));
    }


    public Follower getFollower() {
        return follower;
    }

    public boolean isAutomated() {
        return automatedDrive;
    }
}
