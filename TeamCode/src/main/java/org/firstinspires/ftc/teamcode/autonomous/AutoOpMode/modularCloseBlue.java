package org.firstinspires.ftc.teamcode.autonomous.AutoOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.Paths.closePaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // Assuming Constants class for follower creation

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.DoubleMotorOuttakePID;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;



@Autonomous(name = "Modular Close Blue", group = "Main")
public class modularCloseBlue extends OpMode {

    Intake intake;
    DoubleMotorOuttakePID outtake;

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

    static boolean intake1 = false;
    static boolean intake2 = false;
    static boolean intake3= false;

    private generalStates currentState = generalStates.START;
    int currentSelection = 0;
    int beginningState = 0;

    boolean[] toggles = {false, false, false};   // Path 1, 2, 3
    int cursor = 0;                               // Which item is highlighted

    // Debounce
    boolean upPrev = false;
    boolean downPrev = false;
    boolean aPrev = false;

    @Override
    public void init() {

        intake = new Intake(hardwareMap);
        outtake = new DoubleMotorOuttakePID(hardwareMap);

        // Initialize the follower
        follower = Constants.createFollower(hardwareMap);

        // Path object will be created in start() after routine selection

        // IMPORTANT: Set the robot's starting pose.
        if (isBlueSide()) {
            Pose startPose = new Pose(22, 120, Math.toRadians(135));
            follower.setStartingPose(startPose);
        } else {
            Pose startPose = new Pose(122, 120, Math.toRadians(45));
            follower.setStartingPose(startPose);
        }

        // Initialize timers
        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();

        paths = new closePaths(follower, isBlueSide());
    }

    @Override
    public void init_loop() {
        telemetry.addLine("=== SELECT PATHS ===");
        telemetry.addLine("Use D-Pad Up/Down to move");
        telemetry.addLine("Press A to toggle");
        telemetry.addLine("Press START to confirm\n");

        for (int i = 0; i < 3; i++) {
            String arrow = (i == cursor) ? ">" : " ";
            String state = toggles[i] ? "ON" : "off";
            telemetry.addData(arrow + " Intake " + (i+1), state);
        }

        telemetry.update();

        // Move cursor up
        if (gamepad1.dpad_up && !upPrev) {
            cursor = (cursor - 1 + 3) % 3;
        }

        // Move cursor down
        if (gamepad1.dpad_down && !downPrev) {
            cursor = (cursor + 1) % 3;
        }

        // Toggle current item
        if (gamepad1.a && !aPrev) {
            toggles[cursor] = !toggles[cursor];
        }

        intake1 = toggles[0];
        intake2 = toggles[1];
        intake3 = toggles[2];

        // Update debounce
        upPrev = gamepad1.dpad_up;
        downPrev = gamepad1.dpad_down;
        aPrev = gamepad1.a;
    }

    @Override
    public void start() {
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
        telemetry.addData("Intake1", String.valueOf(intake1));
        telemetry.addData("Intake2", String.valueOf(intake2));
        telemetry.addData("Intake3", String.valueOf(intake3));
        telemetry.addData("Path State", currentState);
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
//        startPath();
        if (intake1) currentSelection = 0;
        else if (intake2) currentSelection = 1;
        else if (intake3) currentSelection = 2;

        cycleRoutine(currentSelection);
    }

    public void startPath() {

    }

    public void cycleRoutine(int pathSelection) {
        switch (currentState) {
            case START:
                switch (beginningState) {
                    case 0:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.Path1);
                            setBeginningState(1);
                        }
                        break;
                    case 1:
                        if(!follower.isBusy()) {
                            outtake.autoRapidShoot(3000,3000, 500);
                            setBeginningState(-1);
                            nextState();
                        }
                        break;
                }
                break;
            case INTAKING:
                switch (pathSelection) {
                    case 0:
                        if (!follower.isBusy()) {
                            intake.autoIntakeOn();
                            follower.followPath(paths.Path2);
                            nextState();
                        }
                        break;
                    case 1:
                        if (!follower.isBusy()) {
                            intake.autoIntakeOn();
                            follower.followPath(paths.Path4);
                            nextState();
                        }
                        break;
                    case 2:
                        if (!follower.isBusy()) {
                            intake.autoIntakeOn();
                            follower.followPath(paths.Path6);
                            nextState();
                        }
                        break;
                }
                break;
            case PRESHOOTING:
                switch (pathSelection) {
                    case 0:
                        if (!follower.isBusy()) {
                            intake.autoIntakeOff();
                            follower.followPath(paths.Path3);
                            nextState();
                        }
                        break;
                    case 1:
                        if (!follower.isBusy()) {
                            intake.autoIntakeOff();
                            follower.followPath(paths.Path5);
                            nextState();
                        }
                        break;
                    case 2:
                        if (!follower.isBusy()) {
                            intake.autoIntakeOff();
                            follower.followPath(paths.Path7);
                            nextState();
                        }
                        break;
                }
                break;
            case SHOOTING:
                if (!follower.isBusy()) {
                    outtake.autoRapidShoot(3000,3000, 500);

                    if (pathSelection == 0) {intake1 = false;}
                    else if (pathSelection == 1) {intake2 = false;}
                    else if (pathSelection == 2) {intake3 = false;}
                }
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

    public void setBeginningState(int state) {
        beginningState = state;
    }

    public void setState(generalStates state) {
        currentState = state;
    }

    // --- Alliance and State Control ---
    boolean isBlueSide() {
        return true;
    }
}
