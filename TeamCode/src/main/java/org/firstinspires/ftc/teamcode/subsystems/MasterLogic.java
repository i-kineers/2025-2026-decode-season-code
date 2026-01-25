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
//    private final DoubleMotorOuttakePID outtake;
    private final FlywheelSystem flywheel;
    private final Intake intake;
    private final AutoAimWithOdometry autoAimWithOdometry;

    private double targetTPS = 1200;
    private double idleTPS = 1000;

    private boolean idleOn = true;

    private boolean dpadUpWasPressed = false;
    private boolean dpadDownWasPressed = false;
    
    // Auto Aim Toggle State
    private boolean autoAimActive = false;
    private boolean previousYState = false;

    private boolean isBlue;

    public MasterLogic(HardwareMap hardwareMap, double startingX, double startingY, double startingH, boolean isBlueAlliance) {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        // Initialize all subsystems
//        outtake = new DoubleMotorOuttakePID(hardwareMap);
        intake = new Intake(hardwareMap);
        flywheel = new FlywheelSystem(hardwareMap);

        if (isBlueAlliance) {
            isBlue = true;
        } else {
            isBlue = false;
        }

        // Initialize Pathing Manager with a default starting pose
        autoAimWithOdometry = new AutoAimWithOdometry(hardwareMap, isBlue);
        autoAimWithOdometry.setStartingPose(startingX,startingY,startingH);
    }
    public void mainLogic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        autoAimWithOdometry.update();

        // --- 1. Drive & Pathing Control ---
        
        // Toggle Auto Aim with Y
        if (gamepad1.y && !previousYState) {
            autoAimActive = !autoAimActive;
        }
        previousYState = gamepad1.y;

        if (gamepad1.xWasPressed()) {
            autoAimWithOdometry.resetAim();
        }

        // X button triggers the automated path defined in PathingManager
        autoAimWithOdometry.drive(
                gamepad1.left_stick_y, // Note: Y stick is usually reversed
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                gamepad1.dpad_up,
                gamepad1.dpad_right,
                gamepad1.dpad_down,
                gamepad1.dpad_left,
                autoAimActive // Auto Aim
        );

        if (autoAimWithOdometry.isAutomated()) {
            targetTPS = autoAimWithOdometry.getCurrentTargetTPS();
            autoAimActive = false; 
        } else {
            // Only update dynamic TPS if NOT in automated pathing mode
            // because pathing mode sets its own targetTPS based on the path
            autoAimWithOdometry.dynamicTargetTPS();
            targetTPS = autoAimWithOdometry.getCurrentTargetTPS();
        }

        if (gamepad1.b) {
            autoAimWithOdometry.resetTargetPose();
        }

        if (gamepad1.left_trigger > 0.1) {
            intake.runIntake(-1); // Intake with Intake
            intake.runGate(0.75);
        }
        else if (gamepad1.left_bumper) {
            intake.runIntake(1); // Outtake with Intake
        } 
        else {
            intake.stopIntake();
            intake.runGate(0);
        }

        if (gamepad1.startWasPressed() && idleOn) {
            idleTPS = 0;
            idleOn = false;
        } else if (gamepad1.startWasPressed() && !idleOn) {
            idleTPS = 1000;
            idleOn = true;
        }

        // Manual control targetTPS
        if (gamepad2.dpad_up && !dpadUpWasPressed) {
            targetTPS += 50;
        }
        if (gamepad2.dpad_down && !dpadDownWasPressed) {
            targetTPS -= 50;
        }
        dpadUpWasPressed = gamepad2.dpad_up;
        dpadDownWasPressed = gamepad2.dpad_down;


        // Reset IMU heading with A button
        if (gamepad1.a) {
            autoAimWithOdometry.resetHeading();
            telemetry.addLine("Heading Reset.");
        }

        flywheel.handleTriggerInput(gamepad1.right_trigger, targetTPS, idleTPS);
        flywheel.update();

        updateTelemetry(telemetry);
    }

    private void updateTelemetry(Telemetry telemetry) {
        // Panels Telemetry (for dashboards)
        Pose currentPose = autoAimWithOdometry.getFollower().getPose();
//        if (currentPose != null) {
//            panelsTelemetry.getTelemetry().addData("Robot X", currentPose.getX());
//            panelsTelemetry.getTelemetry().addData("Robot Y", currentPose.getY());
//            panelsTelemetry.getTelemetry().addData("Robot H", Math.toDegrees(currentPose.getHeading()));
//        }
//        panelsTelemetry.getTelemetry().addData("Target RPM", outtake.getTargetRPM());
//        panelsTelemetry.getTelemetry().addData("Current RPM", outtake.getCurrentRPM());
//        panelsTelemetry.getTelemetry().update();

        // Standard Driver Hub Telemetry
        telemetry.addData("Mode", autoAimWithOdometry.isAutomated() ? "PATHING" : "MANUAL");
        telemetry.addData("Robot State", flywheel.getShotState());
        telemetry.addData("Auto Aim", autoAimActive ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Target TPS", targetTPS);
        telemetry.addData("Actual TPS", flywheel.getVelocity());
        telemetry.addData("Goal X", autoAimWithOdometry.getBackdropPoseX());
        telemetry.addData("Goal Y", autoAimWithOdometry.getBackdropPoseY());
        if (currentPose != null) {
            telemetry.addData("Robot X", currentPose.getX());
            telemetry.addData("Robot Y", currentPose.getY());
            telemetry.addData("Robot H", Math.toDegrees(currentPose.getHeading()));
        }
        telemetry.update();
    }
}
