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

@Autonomous(name = "Modular Close Blue", group = "Main")
public class modularCloseBlue extends OpMode {

    Intake intake;
    DoubleMotorOuttakePID outtake;

    // Pathing and Timers
    private Follower follower;
    private closePaths paths;
    private ElapsedTime pathTimer, opmodeTimer;

    private enum generalStates {
        START,
        INTAKING,
        PRESHOOTING,
        SHOOTING,
        PARKING,
        DONE
    }

    private generalStates currentState = generalStates.START;

    static boolean intake1 = false;
    static boolean intake2 = false;
    static boolean intake3 = false;

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

        // IMPORTANT: Set the robot's starting pose.
        Pose startPose = new Pose(21.192, 121.419, Math.toRadians(135));
        follower.setStartingPose(startPose);

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
            telemetry.addData(arrow + " Intake " + (i + 1), state);
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
        telemetry.addData("Intake1", String.valueOf(intake1));
        telemetry.addData("Intake2", String.valueOf(intake2));
        telemetry.addData("Intake3", String.valueOf(intake3));
        telemetry.addData("Path State", currentState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        if (intake1) currentSelection = 0;
        else if (intake2) currentSelection = 1;
        else if (intake3) currentSelection = 2;
        else if (currentState != generalStates.START && currentState != generalStates.PARKING && currentState != generalStates.DONE) {
            // If no intakes are selected and we are not in a terminal state, park.
            if (!follower.isBusy()) {
                follower.followPath(paths.Path8);
                setState(generalStates.PARKING);
            }
        }

        cycleRoutine(currentSelection);
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
                        if (!follower.isBusy()) {
                            outtake.autoRapidShoot(2700, 5000, 500);
                            setBeginningState(-1);
                            nextState();
                        }
                        break;
                }
                break;
            case INTAKING:
                if (!follower.isBusy()) {
                    intake.autoIntakeOn();
                    switch (pathSelection) {
                        case 0:
                            follower.followPath(paths.Path2);
                            break;
                        case 1:
                            follower.followPath(paths.Path4);
                            break;
                        case 2:
                            follower.followPath(paths.Path6);
                            break;
                    }
                    nextState();
                }
                break;
            case PRESHOOTING:
                if (!follower.isBusy()) {
                    intake.autoIntakeOff();
                    switch (pathSelection) {
                        case 0:
                            follower.followPath(paths.Path3);
                            break;
                        case 1:
                            follower.followPath(paths.Path5);
                            break;
                        case 2:
                            follower.followPath(paths.Path7);
                            break;
                    }
                    nextState();
                }
                break;
            case SHOOTING:
                if (!follower.isBusy()) {
                    outtake.autoRapidShoot(2700, 5000, 500);

                    if (pathSelection == 0) intake1 = false;
                    else if (pathSelection == 1) intake2 = false;
                    else if (pathSelection == 2) intake3 = false;

                    // Check if there are more paths to run
                    if (intake1 || intake2 || intake3) {
                        nextState(); // This will go to INTAKING
                    } else {
                        // All cycles are done, so park.
                        follower.followPath(paths.Path8);
                        setState(generalStates.PARKING);
                    }
                }
                break;
            case PARKING:
                if (!follower.isBusy()) {
                    setState(generalStates.DONE);
                }
                break;
            case DONE:
                // Autonomous is finished, do nothing.
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

    boolean isBlueSide() {
        return true;
    }
}
