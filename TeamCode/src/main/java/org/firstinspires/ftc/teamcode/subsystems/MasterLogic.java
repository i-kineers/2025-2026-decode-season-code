package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * MasterLogic handles the coordination between all subsystems during TeleOp.
 * It encapsulates all the logic so the main OpMode remains clean.
 */
public class MasterLogic {

    private final PanelsTelemetry panelsTelemetry;
    private final DoubleMotorOuttakePID outtake;
    private final Intake intake;
    private final TeleOpPathingManager pathingManager;

    private double targetRPM = 3000;
    private boolean dpadUpWasPressed = false;
    private boolean dpadDownWasPressed = false;

    public MasterLogic(HardwareMap hardwareMap) {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        // Initialize all subsystems
        outtake = new DoubleMotorOuttakePID(hardwareMap);
        intake = new Intake(hardwareMap);
        
        // Initialize Pathing Manager with a default starting pose
        pathingManager = new TeleOpPathingManager(hardwareMap);
        pathingManager.setStartingPose(22,120,135);
//        pathingManager.setTargetPose(48, 95, 135, 0);
    }

    /**
     * The main logic loop to be called in the TeleOp OpMode.
     * @param gamepad1 The primary gamepad
     * @param gamepad2 The secondary gamepad
     * @param telemetry The OpMode's telemetry object
     */
    public void mainLogic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        pathingManager.update();

        // --- 1. Drive & Pathing Control ---
        // X button triggers the automated path defined in PathingManager
        pathingManager.drive(
                gamepad1.left_stick_y, // Note: Y stick is usually reversed
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                gamepad1.dpad_up,
                gamepad1.dpad_right,
                gamepad1.dpad_down,
                gamepad1.dpad_left
        );

        // B button updates the target pose to the current position
//        if (gamepad1.b) {
//            pathingManager.useCurrentPoseForTargetPose();
//        }

        // --- 2. Outtake/Shooter Controls ---

        // Spin up shooter with Right Trigger
        if (gamepad1.right_trigger > 0.1) {
            outtake.setTargetRPM(targetRPM);
        } else {
            outtake.stop();
        }

        // Feed mechanism with Right Bumper
        if (gamepad1.right_bumper) {
            outtake.runLoader();
        } else {
            outtake.stopLoader();
        }

        // --- 3. Intake Controls ---

        // Reverse intake with Left Trigger
        if (gamepad1.left_trigger > 0.1) {
            intake.runIntake(-1);
            intake.runGate(0.75);
        }
        // Forward intake with Left Bumper
        else if (gamepad1.left_bumper) {
            intake.runIntake(1);
        }
        else {
            intake.stopIntake();
            intake.runGate(0);
        }

        // --- 4. Settings & Overrides ---

        // Adjust Target RPM with D-pad (incremental)
        // Note: D-pad is also used for pathing selection in drive() above.
        // You might want to change RPM controls to something else if they conflict.
        // For now, I'll keep them but be aware of the overlap.

        if (gamepad2.dpad_up && !dpadUpWasPressed) {
            targetRPM += 100;
        }
        if (gamepad2.dpad_down && !dpadDownWasPressed) {
            targetRPM -= 100;
        }
        dpadUpWasPressed = gamepad2.dpad_up;
        dpadDownWasPressed = gamepad2.dpad_down;


        // Reset IMU heading with A button
//        if (gamepad1.a) {
//            pathingManager.resetHeading();
//            telemetry.addLine("Heading Reset.");
//        }

        // --- 5. Background Tasks ---
        outtake.update();

        // --- 6. Feedback & Telemetry ---
        updateTelemetry(telemetry);
    }

    private void updateTelemetry(Telemetry telemetry) {
        // Panels Telemetry (for dashboards)
        Pose currentPose = pathingManager.getFollower().getPose();
        if (currentPose != null) {
            panelsTelemetry.getTelemetry().addData("Robot X", currentPose.getX());
            panelsTelemetry.getTelemetry().addData("Robot Y", currentPose.getY());
            panelsTelemetry.getTelemetry().addData("Robot H", Math.toDegrees(currentPose.getHeading()));
        }
        panelsTelemetry.getTelemetry().addData("Target RPM", outtake.getTargetRPM());
        panelsTelemetry.getTelemetry().addData("Current RPM", outtake.getCurrentRPM());
        panelsTelemetry.getTelemetry().update();

        // Standard Driver Hub Telemetry
        telemetry.addData("Mode", pathingManager.isAutomated() ? "PATHING" : "MANUAL");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", outtake.getCurrentRPM());
        telemetry.update();
    }
}
