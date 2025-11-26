package org.firstinspires.ftc.teamcode.autonomous.AutoOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.Paths.closePaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // Assuming Constants class for follower creation

@Autonomous(name = "Modular Close Blue", group = "Main")
public class modularCloseBlue extends OpMode {

    // === AUTO SELECTION ===
    // Use this enum to define the different routines you want to create.
    private enum AutoRoutine {
        FIRST_ROW,
        SECOND_ROW,
        THIRD_ROW,
    }
    private AutoRoutine selectedRoutine = AutoRoutine.CYCLE_THIRD_ROW; // This is the default routine
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    // Pathing and Timers
    private Follower follower;
    private closePaths paths;
    private ElapsedTime pathTimer, opmodeTimer;
    private int pathState;

    @Override
    public void init() {
        // Initialize the follower
        follower = Constants.createFollower(hardwareMap);

        // Create an instance of our path library
        paths = new closePaths(follower, 1, isBlueSide());

        // IMPORTANT: Set the robot's starting pose.
        Pose startPose = new Pose(22, 120, Math.toRadians(135));
        follower.setStartingPose(startPose);

        // Initialize timers
        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
    }

    @Override
    public void init_loop() {
        // --- USER SELECTION LOGIC ---
        telemetry.addLine("=== AUTO SELECTION ===");
        telemetry.addLine("Use D-Pad Up/Down to cycle through routines.");
        telemetry.addData("Selected Routine", selectedRoutine);
        telemetry.addLine("\nPress START when ready.");

        // Cycle up through routines
        if (gamepad1.dpad_up && !dpadUpPressed) {
            int nextIndex = selectedRoutine.ordinal() - 1;
            if (nextIndex < 0) nextIndex = AutoRoutine.values().length - 1;
            selectedRoutine = AutoRoutine.values()[nextIndex];
        }

        // Cycle down through routines
        if (gamepad1.dpad_down && !dpadDownPressed) {
            int nextIndex = (selectedRoutine.ordinal() + 1) % AutoRoutine.values().length;
            selectedRoutine = AutoRoutine.values()[nextIndex];
        }

        // Debounce the D-pad buttons
        dpadUpPressed = gamepad1.dpad_up;
        dpadDownPressed = gamepad1.dpad_down;

        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(0);
    }

    @Override
    public void stop() {
        // This can be left empty
    }

    @Override
    public void loop() {
        // These must be called continuously for path following to work
        follower.update();
        autonomousPathUpdate(); // Run the main state machine

        // Feedback to Driver Hub
        telemetry.addData("Running Routine", selectedRoutine);
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /**
     * This is the foundational state machine. Build your autonomous logic inside this method.
     * Use the 'selectedRoutine' variable to decide which paths to run.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // This is the first step. It's often the same for all routines,
                // like moving to a shooting position.
                follower.followPath(paths.Path1);
                setPathState(1);
                break;

            case 1:
                // This is the second step. The robot has arrived at the first location.
                if (!follower.isBusy()) {
                    // Now, use the user's selection to decide the next move.
                    switch (selectedRoutine) {
                        case CYCLE_THIRD_ROW:
                            follower.followPath(paths.shootToPickup3);
                            setPathState(2); // Continue to the next state for this routine
                            break;
                        case CYCLE_FIRST_ROW:
                            follower.followPath(paths.shootToPickup1);
                            setPathState(2); // Continue to the next state for this routine
                            break;
                        case PARK_ONLY:
                            follower.followPath(paths.shootToHome);
                            setPathState(100); // Go to a final "end" state
                            break;
                    }
                }
                break;

            case 2:
                // This state is for the cycling routines.
                if (!follower.isBusy()) {
                    switch (selectedRoutine) {
                        case CYCLE_THIRD_ROW:
                            follower.followPath(paths.pickup3ToShoot);
                            break;
                        case CYCLE_FIRST_ROW:
                            follower.followPath(paths.pickup1ToShoot);
                            break;
                    }
                    setPathState(3);
                }
                break;

            case 3:
                // This is the final movement for the cycling routines (parking).
                if (!follower.isBusy()) {
                    follower.followPath(paths.shootToHome);
                    setPathState(100); // Go to a final "end" state
                }
                break;

            case 100: // This is a shared "end" state.
                if (!follower.isBusy()) {
                    setPathState(-1); // Set state to -1 to signify the routine is complete.
                }
                break;
        }
    }

    // --- Alliance and State Control ---
    protected boolean isBlueSide() {
        return true;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }
}
