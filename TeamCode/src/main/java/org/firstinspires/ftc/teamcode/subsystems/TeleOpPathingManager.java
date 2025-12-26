package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autonomous.Paths.teleopPath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;


public class TeleOpPathingManager {
    private Follower follower;
    private boolean automatedDrive = false;

    private Pose targetPose;
    private Pose targetPose2;
    private Pose targetPose3;
    private Pose targetPose4;
    private Pose startingPose;

    private List<Pose> targetPoseList;

    public TeleOpPathingManager(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);

        // Default starting pose if not set before init
        startingPose = new Pose(0, 0, 0);
        follower.setStartingPose(startingPose);
        follower.startTeleopDrive();

        // Default target pose
        this.targetPose = new Pose(48, 95, Math.toRadians(135));
        this.targetPose2 = new Pose(83.371, 130, Math.toRadians(180));
        this.targetPose3 = new Pose(90.318, 91.981, Math.toRadians(147));
        this.targetPose4 = new Pose(72, 72, Math.toRadians(135));

        targetPoseList = new ArrayList<>();
        targetPoseList.add(targetPose);
        targetPoseList.add(targetPose2);
        targetPoseList.add(targetPose3);
        targetPoseList.add(targetPose4);
    }

    public void update() {
        follower.update();
    }

    /**
     * Handles TeleOp driving and automated path triggering.
     * @param forward Forward movement (e.g. gamepad.left_stick_y)
     * @param strafe Strafe movement (e.g. gamepad.left_stick_x)
     * @param turn Turn movement (e.g. -gamepad.right_stick_x)
     */
    public void drive(double forward, double strafe, double turn,
                      boolean dpadUp, boolean dpadRight,
                      boolean dpadDown, boolean dpadLeft) {

        boolean manualInput = Math.abs(forward) > 0.1
                || Math.abs(strafe) > 0.1
                || Math.abs(turn) > 0.1;

        // Trigger Pathing based on D-pad
        if (!automatedDrive) {

            int selectedIndex = -1;

            if (dpadUp) {
                selectedIndex = 0;
            } else if (dpadRight) {
                selectedIndex = 1;
            } else if (dpadDown) {
                selectedIndex = 2;
            } else if (dpadLeft) {
                selectedIndex = 3;
            }

            if (selectedIndex != -1 && targetPoseList != null
                    && selectedIndex < targetPoseList.size()) {

                Pose selectedTarget = targetPoseList.get(selectedIndex);

                if (selectedTarget != null) {
                    PathChain path = teleopPath.getPath(follower, selectedTarget);
                    follower.followPath(path);
                    automatedDrive = true;
                }
            }
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


    public void setStartingPose(double x, double y, double h) {
        startingPose = new Pose(x, y, Math.toRadians(h));
        // Update follower's starting pose immediately if this is called
        if (follower != null) {
            follower.setStartingPose(startingPose);
        }
    }

    public void setTargetPose(double x, double y, double h, int targetSelection) {
        Pose newPose = new Pose(x, y, Math.toRadians(h));

        // Always keep targetPose behaving the same
        if (targetSelection == 0) {
            targetPose = newPose;
        }

        // Update list entry if valid
        if (targetPoseList != null
                && targetSelection >= 0
                && targetSelection < targetPoseList.size()) {

            targetPoseList.set(targetSelection, newPose);
        }
    }


    public void resetTargetPose() {
        if (follower.getPose() != null) {
            // Default to updating the first target pose (index 0) if B is pressed
            // You can change this logic to update the last selected index if you track it
            Pose current = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());


            targetPose = current;
            if (targetPoseList != null && !targetPoseList.isEmpty()) {
                targetPoseList.set(0, current);
            }
        }
    }

    /**
     * Resets the robot's current heading to 0 degrees (facing forward).
     */
    public void resetHeading() {
        if (follower != null) {
            Pose currentPose = follower.getPose();
            if (currentPose != null) {
                // Create a new pose with the same X and Y, but a heading of 0
                follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), 0));
            }
        }
    }

    public Follower getFollower() {
        return follower;
    }

    public Pose getStartingPose() {
        return startingPose;
    }

    public boolean isAutomated() {
        return automatedDrive;
    }
}
