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

    // Base default poses (constants)
    private final Pose CLOSE_ONE = new Pose(48, 95, Math.toRadians(135));
    private final Pose CLOSE_TWO = new Pose(100.295, 99.999, Math.toRadians(160));
    private final Pose FAR_ONE = new Pose(55.580, 11.282, Math.toRadians(115));
    private final Pose FAR_TWO = new Pose(85, 11.461, Math.toRadians(120.8));

    private final List<Pose> defaultTargets = new ArrayList<>();
    
    // RPMs corresponding to each target
    private final double[] targetRPMs = {2700, 3000, 3500, 4000};
    private double currentTargetRPM = 3000; // Default

    private Pose startingPose;
    private List<Pose> targetPoseList;

    public TeleOpPathingManager(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);

        // Default starting pose if not set before init
        startingPose = new Pose(0, 0, 0);
        follower.setStartingPose(startingPose);
        follower.startTeleopDrive();

        // Initialize default targets list for easy iteration
        defaultTargets.add(CLOSE_ONE);
        defaultTargets.add(CLOSE_TWO);
        defaultTargets.add(FAR_ONE);
        defaultTargets.add(FAR_TWO);

        // Initialize list with default poses
        targetPoseList = new ArrayList<>();
        for (Pose p : defaultTargets) {
            targetPoseList.add(new Pose(p.getX(), p.getY(), p.getHeading()));
        }
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
                selectedIndex = 0; // Close 1
            } else if (dpadLeft) {
                selectedIndex = 1; // Close 2
            } else if (dpadRight) {
                selectedIndex = 2; // Far 1
            } else if (dpadDown) {
                selectedIndex = 3; // Far 2
            }

            if (selectedIndex != -1 && targetPoseList != null
                    && selectedIndex < targetPoseList.size()) {

                Pose selectedTarget = targetPoseList.get(selectedIndex);
                
                // Update the target RPM based on selection
                currentTargetRPM = targetRPMs[selectedIndex];

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

        // Update list entry if valid
        if (targetPoseList != null
                && targetSelection >= 0
                && targetSelection < targetPoseList.size()) {

            targetPoseList.set(targetSelection, newPose);
        }
    }


    /**
     * Resets all target poses based on the offset between the current robot pose
     * and the CLOSEST default target pose.
     */
    public void resetTargetPose() {
        if (follower.getPose() != null) {
            Pose currentPose = follower.getPose();
            
            // Find the closest default target
            int closestIndex = -1;
            double minDistance = Double.MAX_VALUE;

            for (int i = 0; i < defaultTargets.size(); i++) {
                Pose target = defaultTargets.get(i);
                double dist = Math.hypot(currentPose.getX() - target.getX(), currentPose.getY() - target.getY());
                if (dist < minDistance) {
                    minDistance = dist;
                    closestIndex = i;
                }
            }

            if (closestIndex != -1) {
                Pose closestDefault = defaultTargets.get(closestIndex);

                // Calculate offset: Current Pose - Closest Default Target
                double offsetX = currentPose.getX() - closestDefault.getX();
                double offsetY = currentPose.getY() - closestDefault.getY();
                double offsetH = currentPose.getHeading() - closestDefault.getHeading();

                // Apply this offset to ALL targets relative to their defaults
                for (int i = 0; i < targetPoseList.size(); i++) {
                    targetPoseList.set(i, applyOffset(defaultTargets.get(i), offsetX, offsetY, offsetH));
                }
            }
        }
    }
    
    private Pose applyOffset(Pose base, double offX, double offY, double offH) {
        return new Pose(
            base.getX() + offX,
            base.getY() + offY,
            base.getHeading() + offH
        );
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
    
    public double getCurrentTargetRPM() {
        return currentTargetRPM;
    }

    public boolean isAutomated() {
        return automatedDrive;
    }
}
