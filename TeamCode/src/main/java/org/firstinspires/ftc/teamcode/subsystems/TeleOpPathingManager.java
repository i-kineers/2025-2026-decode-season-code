package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFController;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.teamcode.autonomous.Paths.teleopPath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;


public class TeleOpPathingManager {
    private Follower follower;
    private boolean automatedDrive = false;

    // Auto Aim
    private PIDFController headingController;
    private Pose goalPose = new Pose(8.472, 139.091);
    private boolean wasAutoAim = false;

    // Base default poses (constants)
    private Pose CLOSE_ONE = new Pose(48, 95, Math.toRadians(135));
    private Pose CLOSE_TWO = new Pose(100.295, 99.999, Math.toRadians(160));
    private Pose FAR_ONE = new Pose(55.580, 11.282, Math.toRadians(110));
    private Pose FAR_TWO = new Pose(85, 11.461, Math.toRadians(120.8));

    private final List<Pose> defaultTargets = new ArrayList<>();
    
    // RPMs corresponding to each target
    private final double[] targetRPMs = {1213, 1400, 1773, 1820};
    private double currentTargetTPS = 1200; // Default

    private Pose startingPose;
    private List<Pose> targetPoseList;

    private boolean isBlue;

    public TeleOpPathingManager(HardwareMap hardwareMap, boolean isBlueAlliance) {
        follower = Constants.createFollower(hardwareMap);
        headingController = new PIDFController(follower.constants.coefficientsHeadingPIDF);

        // Default starting pose if not set before init
        startingPose = new Pose(0, 0, 0);
        follower.setStartingPose(startingPose);
        follower.startTeleopDrive();

        // Check if Blue or Red alliance
        if (!isBlueAlliance) { isBlue = false; } else { isBlue = true; }

        if (!isBlue) {
            CLOSE_ONE = CLOSE_ONE.mirror();
            CLOSE_TWO = CLOSE_TWO.mirror();
            FAR_ONE = FAR_ONE.mirror();
            FAR_TWO = FAR_TWO.mirror();
            goalPose = goalPose.mirror();
        }

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
                      boolean dpadDown, boolean dpadLeft, boolean autoAim) {

        boolean manualInput = Math.abs(forward) > 0.1
                || Math.abs(strafe) > 0.1
                || Math.abs(turn) > 0.1;
        
        if (autoAim) manualInput = true;

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
                currentTargetTPS = targetRPMs[selectedIndex];

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
            double turnPower = turn;

            if (autoAim) {
                if (!wasAutoAim) {
                    headingController.reset();
                }

                Pose robot = follower.getPose();
                double dx = goalPose.getX() - robot.getX();
                double dy = goalPose.getY() - robot.getY();
                double targetHeading = MathFunctions.normalizeAngle(Math.atan2(dy, dx));

                double currentHeading = robot.getHeading();
                double error = targetHeading - currentHeading;

                // Normalize to [-π, π]
                if (error > Math.PI) {
                    error -= 2 * Math.PI;
                } else if (error < -Math.PI) {
                    error += 2 * Math.PI;
                }

                if (Math.abs(error) < Math.toRadians(1.5)) {
                    turnPower = 0;
                } else {
                    headingController.updateError(error);
                    turnPower = headingController.run();

                    // Ramp down turn power near target
                    double maxTurn = 0.4;
                    if (Math.abs(error) < Math.toRadians(10)) {
                        maxTurn = 0.2;
                    }

                    turnPower = MathFunctions.clamp(turnPower, -maxTurn, maxTurn);
                }
            }
            wasAutoAim = autoAim;

            if (isBlue) {
                follower.setTeleOpDrive(forward, strafe, turnPower, false);
            } else {
                follower.setTeleOpDrive(-forward, -strafe, turnPower, false);
            }
        } else {
            wasAutoAim = false;
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
    
    public double getCurrentTargetTPS() {
        return currentTargetTPS;
    }

    public boolean isAutomated() {
        return automatedDrive;
    }

    public boolean isAutoAiming() {
        return wasAutoAim;
    }
}
