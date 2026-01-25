package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autonomous.Paths.closePaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static android.os.SystemClock.sleep;

public class AutonomousCycleManager {

    // Subsystems
    private final Follower follower;
    private final Intake intake;
    private final DoubleMotorOuttakePID outtake;
    private final FlywheelSystem flywheelSystem;
    private final closePaths paths;

    // State Machine
    private enum GeneralStates {
        START,
        INTAKING,
        PRESHOOTING,
        SHOOTING,
        PARKING,
        DONE
    }

    private GeneralStates currentState = GeneralStates.START;
    private int beginningState = 0;
    private int currentSelection = 0;

    // Path Selection
    private boolean intake1 = false;
    private boolean intake2 = false;
    private boolean intake3 = false;
    private boolean useGate = false;
    private boolean gateTriggered = false;

    private Pose blueStartPose = new Pose(22,120, Math.toRadians(135));
    private Pose redStartPose = new Pose(122, 120, Math.toRadians(45));

    public AutonomousCycleManager(HardwareMap hardwareMap, boolean isBlueSide) {
        // Initialize subsystems
        intake = new Intake(hardwareMap);
        outtake = new DoubleMotorOuttakePID(hardwareMap);
        flywheelSystem = new FlywheelSystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        paths = new closePaths(follower, isBlueSide);

        // Set starting pose
        if (isBlueSide) {
            follower.setStartingPose(blueStartPose);
        } else {
            follower.setStartingPose(redStartPose);
        }

    }

    /**
     * Sets which intake cycles to run.
     * @param runIntake1 Run the first intake path.
     * @param runIntake2 Run the second intake path.
     * @param runIntake3 Run the third intake path.
     */
    public void setCycles(boolean runIntake1, boolean runIntake2, boolean runIntake3, boolean openGate) {
        this.intake1 = runIntake1;
        this.intake2 = runIntake2;
        this.intake3 = runIntake3;
        this.useGate = openGate;

    }

    /**
     * Main update loop to be called from the OpMode.
     */
    public void update() {
        follower.update();
        cycleRoutine();
    }

    private void cycleRoutine() {
        // Determine which cycle to run next if we are in a waiting state
        if (currentState == GeneralStates.START || (currentState == GeneralStates.INTAKING && !follower.isBusy())) {
            if (intake1) currentSelection = 0;
            else if (intake2) currentSelection = 1;
            else if (intake3) currentSelection = 2;
            else {
                // If no cycles are selected or left, and we are not in the start state, go to park.
                if (currentState != GeneralStates.START) {
                    currentState = GeneralStates.PARKING;
                    follower.followPath(paths.Path8);
                    return;
                }
            }
        }

        switch (currentState) {
            case START:
                if (beginningState == 0 && !follower.isBusy()) {
                    follower.followPath(paths.Path1);
                    beginningState = 1;
                } else if (beginningState == 1 && !follower.isBusy()) {
                    flywheelSystem.autoRapidShoot(1200, 3000, 500);
                    beginningState = -1; // Mark as done
                    nextState();
                }
                break;

            case INTAKING:
                if (!follower.isBusy()) {
                    intake.autoIntakeOn();
                    if (currentSelection == 0) follower.followPath(paths.Path2);
                    else if (currentSelection == 1) follower.followPath(paths.Path4);
                    else if (currentSelection == 2) follower.followPath(paths.Path6);
                    nextState();
                }
                break;

            case PRESHOOTING:
                if (!follower.isBusy()) {
                    intake.autoIntakeOff();
                    follower.setMaxPower(1.0);

                    if (currentSelection == 0) {
                        // Check if we need to START the gate path
                        if (useGate) {
                            follower.setMaxPower(0.4);
                            follower.followPath(paths.Path9);
                            useGate = false;        // "Consumes" the instruction from the OpMode
                            gateTriggered = true;   // REMEMBERS we are currently in the gate sequence
                        }
                        // If we are already mid-gate sequence, return to shoot
                        else if (gateTriggered) {
                            flywheelSystem.sleep(500);
                            follower.setMaxPower(1.0);
                            follower.followPath(paths.Path10);
                            gateTriggered = false;  // Reset for next time
                            nextState();
                        }
                        // Normal flow: No gate was ever requested
                        else {
                            follower.followPath(paths.Path3);
                            nextState();
                        }
                    } else {
                        // Normal logic for Row 2 and 3
                        if (currentSelection == 1) follower.followPath(paths.Path5);
                        else if (currentSelection == 2) follower.followPath(paths.Path7);
                        nextState();
                    }
                }
                break;

            case SHOOTING:
                if (!follower.isBusy()) {
                    intake.autoIntakeOn();
                    sleep(400);
                    intake.autoIntakeOff();
                    flywheelSystem.autoRapidShoot(1200, 3000, 500);

                    // Mark current task as done
                    if (currentSelection == 0) intake1 = false;
                    else if (currentSelection == 1) intake2 = false;
                    else if (currentSelection == 2) intake3 = false;

                    // Decide what to do next
                    if (intake1 || intake2 || intake3) {
                        currentState = GeneralStates.INTAKING; // More cycles to run
                    } else {
                        currentState = GeneralStates.PARKING; // All cycles done, park
                        follower.followPath(paths.Path8);
                    }
                }
                break;

            case PARKING:
                if (!follower.isBusy()) {
                    currentState = GeneralStates.DONE; // Final state
                }
                break;

            case DONE:
                // OpMode will idle here until it's stopped.
                break;
        }
    }

    private void nextState() {
        switch (currentState) {
            case START:
                currentState = GeneralStates.INTAKING;
                break;
            case INTAKING:
                currentState = GeneralStates.PRESHOOTING;
                break;
            case PRESHOOTING:
                currentState = GeneralStates.SHOOTING;
                break;
            case SHOOTING:
                // This is handled inside the SHOOTING case now
                break;
        }
    }

    // Expose follower and state for telemetry
    public Follower getFollower() { return follower; }
    public String getCurrentState() { return currentState.toString(); }
}
