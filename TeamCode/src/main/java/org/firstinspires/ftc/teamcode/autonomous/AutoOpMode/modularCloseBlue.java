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

    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    // Pathing and Timers
    private Follower follower;
    private closePaths paths;
    private ElapsedTime pathTimer, opmodeTimer;

    private enum pathState {
        FIRST_ROW,
        SECOND_ROW,
        THIRD_ROW,
    }

    private enum generalStates {
        START,
        INTAKING,
        PRESHOOTING,
        SHOOTING
    }

    private generalStates currentState = generalStates.START;
//    private int beginningState = 0;

    @Override
    public void init() {
        // Initialize the follower
        follower = Constants.createFollower(hardwareMap);

        // Path object will be created in start() after routine selection

        // IMPORTANT: Set the robot's starting pose.
        Pose startPose = new Pose(22, 120, Math.toRadians(135));
        follower.setStartingPose(startPose);

        // Initialize timers
        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
    }

    @Override
    public void init_loop() {
//        // --- USER SELECTION LOGIC ---
//        telemetry.addLine("=== AUTO SELECTION ===");
//        telemetry.addLine("Use D-Pad Up/Down to cycle through routines.");
//        telemetry.addData("Selected Routine", selectedRoutine);
//        telemetry.addLine(" Press START when ready.");
//
//        // Cycle up through routines
//        if (gamepad1.dpad_up && !dpadUpPressed) {
//            int prevIndex = (selectedRoutine.ordinal() - 1 + AutoRoutine.values().length) % AutoRoutine.values().length;
//            selectedRoutine = AutoRoutine.values()[prevIndex];
//        }
//
//        // Cycle down through routines
//        if (gamepad1.dpad_down && !dpadDownPressed) {
//            int nextIndex = (selectedRoutine.ordinal() + 1) % AutoRoutine.values().length;
//            selectedRoutine = AutoRoutine.values()[nextIndex];
//        }
//
//        // Debounce the D-pad buttons
//        dpadUpPressed = gamepad1.dpad_up;
//        dpadDownPressed = gamepad1.dpad_down;
//
//        telemetry.update();
    }

    @Override
    public void start() {
//        int pathSelection = 1; // Default
//        switch (selectedRoutine) {
//            case FIRST_ROW:
//            case CYCLE_FIRST_ROW:
//                pathSelection = 1;
//                break;
//            case SECOND_ROW:
//                // Assuming second row corresponds to pathSelection = 2
//                pathSelection = 2;
//                break;
//            case THIRD_ROW:
//            case CYCLE_THIRD_ROW:
//                // Assuming third row corresponds to pathSelection = 3
//                pathSelection = 3;
//                break;
//        }
        paths = new closePaths(follower, true);

        opmodeTimer.reset();
    }

    @Override
    public void stop() {
        // This can be left empty
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
//        telemetry.addData("Running Routine", selectedRoutine);
//        telemetry.addData("Master State", masterState);
//        telemetry.addData("Path State", pathState);
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

    }

    public void cycleRoutine(int pathSelection) {
        switch (currentState) {
            case START:
                follower.followPath(paths.Path1);
//                setBeginningState(-1);
                break;
            case INTAKING:
                switch (pathSelection) {
                    case 0:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.Path2);
                        }
                        break;
                    case 1:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.Path4);
                        }
                        break;
                    case 2:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.Path6);
                        }
                        break;
                }
                break;
            case PRESHOOTING:
                switch (pathSelection) {
                    case 0:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.Path3);
                        }
                        break;
                    case 1:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.Path5);
                        }
                        break;
                    case 2:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.Path7);
                        }
                        break;
                }
                break;
            case SHOOTING:
                // Shooting logic goes here
                break;
        }
    }

    public void nextState() {
        switch (currentState) {
            case START:
                currentState = generalStates.INTAKING;
                break;
            case INTAKING:
                currentState = generalStates.PRESHOOTING;
                break;
            case PRESHOOTING:
                currentState = generalStates.SHOOTING;
                break;
            case SHOOTING:
                currentState = generalStates.INTAKING;
                break;
        }
    }

//    public void setBeginningState(int state) {
//        beginningState = state;
//    }

    public void setState(generalStates state) {
        currentState = state;
    }

    // --- Alliance and State Control ---
    boolean isBlueSide() {
        return true;
    }
}
