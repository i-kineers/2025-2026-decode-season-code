package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

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
    private final DoubleIntake intake;
    private final AutoAimWithOdometry autoAimWithOdometry;

    private double targetTPS = 1200;

    // Auto Aim Toggle State
    private boolean autoAimActive = false;
    private boolean previousYState = false;

    private boolean isBlue;

    public MasterLogic(HardwareMap hardwareMap, double startingX, double startingY, double startingH, boolean isBlueAlliance) {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        // Initialize all subsystems
        intake = new DoubleIntake(hardwareMap);
        flywheel = new FlywheelSystem(hardwareMap);

        isBlue = isBlueAlliance;

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

        autoAimWithOdometry.drive(gamepad1, autoAimActive);

        if (autoAimWithOdometry.isAutomated()) {
            targetTPS = autoAimWithOdometry.getCurrentTargetTPS();
            autoAimActive = false;
        } else {
            autoAimWithOdometry.dynamicTargetTPS();
            targetTPS = autoAimWithOdometry.getCurrentTargetTPS();
        }

        if (gamepad1.b) {
            autoAimWithOdometry.resetTargetPose();
        }

        intake.runIntake(gamepad1);
        flywheel.runFlywheel(gamepad1, gamepad2);

        flywheel.setNormalTPS(targetTPS);

        // Reset IMU heading with A button
        if (gamepad1.a) {
            autoAimWithOdometry.resetHeading();
            telemetry.addLine("Heading Reset.");
        }

        updateTelemetry(telemetry);
    }

    private void updateTelemetry(Telemetry telemetry) {
        Pose currentPose = autoAimWithOdometry.getFollower().getPose();

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
