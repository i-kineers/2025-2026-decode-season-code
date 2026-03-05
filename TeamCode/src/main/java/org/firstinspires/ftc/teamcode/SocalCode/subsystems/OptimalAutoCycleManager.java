package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SocalCode.autonomous.Paths.optimalClosePaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class OptimalAutoCycleManager {

    private final Follower follower;
    private final DoubleIntake intake;
    private final FlywheelSystem flywheelSystem;
    private final optimalClosePaths paths;

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

    private boolean spike2 = false;
    private boolean intakeGate = false;
    private boolean spike1 = false;

    private boolean shootingInitialized = false;
    private static boolean runKickers = false;

    private final ElapsedTime shotTimer = new ElapsedTime();

    private Pose blueStartPose = new Pose(19.5,122.6, Math.toRadians(135));
    private Pose redStartPose = new Pose(124.5,122.6, Math.toRadians(45));

    public OptimalAutoCycleManager(HardwareMap hardwareMap, boolean isBlue) {

        intake = new DoubleIntake(hardwareMap);
        flywheelSystem = new FlywheelSystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        paths = new optimalClosePaths(follower, isBlue);

        if (isBlue) {
            follower.setStartingPose(blueStartPose);
        } else {
            follower.setStartingPose(redStartPose);
        }

        // Start spinning flywheel immediately for fast first shot
        flywheelSystem.setTargetTPS(1100);
    }

    public void setCycles(boolean SPIKETWO, boolean INTAKEGATE, boolean SPIKEONE) {
        spike2 = SPIKETWO;
        intakeGate = INTAKEGATE;
        spike1 = SPIKEONE;
    }

    public void update() {
        follower.update();
        flywheelSystem.autoShootLogic(runKickers);
        intake.autoIntakeOn(true); // adjust if needed per side
        cycleRoutine();
    }

    private void cycleRoutine() {

        if (currentState == GeneralStates.START || (currentState == GeneralStates.INTAKING && !follower.isBusy())) {
            if (spike2) currentSelection = 0;
            else if (intakeGate) currentSelection = 1;
            else if (spike1) currentSelection = 2;
            else {
                if (currentState != GeneralStates.START) {
                    currentState = GeneralStates.PARKING;
                    follower.followPath(paths.Path10);
                    return;
                }
            }
        }

        switch (currentState) {

            case START:
                if (beginningState == 0 && !follower.isBusy()) {
                    follower.followPath(paths.Path1);
                    beginningState = 1;
                }
                else if (beginningState == 1 && !follower.isBusy() && handleShooting()) {
                    beginningState = -1;
                    nextState();
                }
                break;

            case INTAKING:
                if (!follower.isBusy()) {

                    intake.setAutoIntakeState(DoubleIntake.autoIntakeState.INTAKE);

                    if (currentSelection == 0)
                        follower.followPath(paths.Path2);
                    else if (currentSelection == 1)
                        follower.followPath(paths.Path4);
                    else if (currentSelection == 2)
                        follower.followPath(paths.Path6);
                    else if (currentSelection == 3)
                        follower.followPath(paths.Path8);

                    nextState();
                }
                break;

            case PRESHOOTING:
                if (!follower.isBusy()) {

                    intake.setAutoIntakeState(DoubleIntake.autoIntakeState.IDLE);

                    if (currentSelection == 0)
                        follower.followPath(paths.Path3);
                    else if (currentSelection == 1)
                        follower.followPath(paths.Path5);
                    else if (currentSelection == 2)
                        follower.followPath(paths.Path7);

                    nextState();
                }
                break;

            case SHOOTING:
                if (!follower.isBusy() && handleShooting()) {

                    if (currentSelection == 0) spike2 = false;
                    else if (currentSelection == 1) intakeGate = false;
                    else if (currentSelection == 2) spike1 = false;

                    if (spike2 || intakeGate || spike1)
                        currentState = GeneralStates.INTAKING;
                    else {
                        currentState = GeneralStates.PARKING;
                        follower.followPath(paths.Path10);
                    }
                }
                break;

            case PARKING:
                if (!follower.isBusy()) {
                    flywheelSystem.setTargetTPS(0);
                    currentState = GeneralStates.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    private void nextState() {
        switch (currentState) {
            case START: currentState = GeneralStates.INTAKING; break;
            case INTAKING: currentState = GeneralStates.PRESHOOTING; break;
            case PRESHOOTING: currentState = GeneralStates.SHOOTING; break;
        }
    }

    private boolean handleShooting() {

        if (!shootingInitialized) {
            shotTimer.reset();
            intake.setAutoIntakeState(DoubleIntake.autoIntakeState.SHOOTING);
            shootingInitialized = true;
        }

        if (shotTimer.milliseconds() < 2500) {
            runKickers = true;
            return false;
        } else {
            runKickers = false;
            shootingInitialized = false;
            return true;
        }
    }

    public Follower getFollower() { return follower; }
    public String getCurrentState() { return currentState.toString(); }
}