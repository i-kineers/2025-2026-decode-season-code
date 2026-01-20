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
    private final FlywheelSystem flywheel;
    private final Intake intake;
    private final TeleOpPathingManager pathingManager;

    private double targetTPS = 1200;
    private boolean dpadUpWasPressed = false;
    private boolean dpadDownWasPressed = false;
    
    // Auto Aim Toggle State
    private boolean autoAimActive = false;
    private boolean previousYState = false;

    private boolean isBlue;

    public MasterLogic(HardwareMap hardwareMap, double startingX, double startingY, double startingH, boolean isBlueAlliance) {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        intake = new Intake(hardwareMap);
        flywheel = new FlywheelSystem(hardwareMap);

        if (isBlueAlliance) {
            isBlue = true;
        } else {
            isBlue = false;
        }

        pathingManager = new TeleOpPathingManager(hardwareMap, isBlue);
        pathingManager.setStartingPose(startingX,startingY,startingH);
    }
    public void mainLogic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        pathingManager.update();

        if (gamepad1.y && !previousYState) {
            autoAimActive = !autoAimActive;
        }
        previousYState = gamepad1.y;

        pathingManager.drive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                gamepad1.dpad_up,
                gamepad1.dpad_right,
                gamepad1.dpad_down,
                gamepad1.dpad_left,
                autoAimActive
        );

        if (pathingManager.isAutomated()) {
            targetTPS = pathingManager.getCurrentTargetTPS();
            autoAimActive = false; 
        }

        if (gamepad1.b) {
            pathingManager.resetTargetPose();
        }

        if (gamepad1.right_trigger > 0.1) {
            flywheel.setTargetTPS(targetTPS);
        } else {
            flywheel.setTargetTPS(0);
        }

        if (gamepad1.right_bumper) {
            flywheel.runLoader();
        } else {
            flywheel.stopLoader();
        }

        if (gamepad1.left_trigger > 0.1) {
            intake.runIntake(-1);
            intake.runGate(0.75);
        }
        else if (gamepad1.left_bumper) {
            intake.runIntake(1);
        } 
        else {
            intake.stopIntake();
            intake.runGate(0);
        }

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
            pathingManager.resetHeading();
            telemetry.addLine("Heading Reset.");
        }

        flywheel.update();
        updateTelemetry(telemetry);
    }

    private void updateTelemetry(Telemetry telemetry) {
        Pose currentPose = pathingManager.getFollower().getPose();

        telemetry.addData("Mode", pathingManager.isAutomated() ? "PATHING" : "MANUAL");
        telemetry.addData("Auto Aim", autoAimActive ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Target RPM", targetTPS);
        telemetry.addData("Actual RPM", flywheel.getVelocity());
        if (currentPose != null) {
            telemetry.addData("Robot X", currentPose.getX());
            telemetry.addData("Robot Y", currentPose.getY());
            telemetry.addData("Robot H", Math.toDegrees(currentPose.getHeading()));
        }
        telemetry.update();
    }
}
