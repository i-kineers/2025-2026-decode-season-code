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
//    private final FlywheelSystem flywheel;
    private final DoubleIntake intake;
    private final AutoAimWithOdometry autoAimWithOdometry;

    private double targetTPS = 1200;

    // Auto Aim Toggle State
    private boolean autoAimActive = false;

    private boolean isBlue;

    private double goalDist;

    public MasterLogic(HardwareMap hardwareMap, double startingX, double startingY, double startingH, boolean isBlueAlliance) {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        intake = new DoubleIntake(hardwareMap);
        autoAimWithOdometry = new AutoAimWithOdometry(hardwareMap, isBlue);
        autoAimWithOdometry.setStartingPose(startingX,startingY,startingH);

        isBlue = isBlueAlliance;
    }

    public void mainLogic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        autoAimWithOdometry.update();

        if (gamepad1.right_bumper) {
            autoAimActive = true;
        } else {
            autoAimActive = false;
        }

        if (gamepad1.xWasPressed()) {
            autoAimWithOdometry.resetAim();
        }

        autoAimWithOdometry.drive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                gamepad1.dpad_up,
                gamepad1.dpad_right,
                gamepad1.dpad_down,
                gamepad1.dpad_left,
                autoAimActive
        );

        if (autoAimWithOdometry.isAutomated()) {
            targetTPS = autoAimWithOdometry.getCurrentTargetTPS();
            autoAimActive = false;
        } else {
            goalDist = autoAimWithOdometry.getDistanceFromGoal();
            targetTPS = autoAimWithOdometry.newDynamicTargetTPS(goalDist);
        }

        if (gamepad1.b) {
            autoAimWithOdometry.resetTargetPose();
        }

        // Reset IMU heading with A button
//        if (gamepad1.a) {
//            autoAimWithOdometry.resetHeading();
//            telemetry.addLine("Heading Reset.");
//        }

        intake.runIntake(gamepad1);

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
//        telemetry.addData("Robot State", flywheel.getShotState());
        telemetry.addData("Auto Aim", autoAimActive ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Target TPS", targetTPS);
//        telemetry.addData("Actual TPS", flywheel.getVelocity());
        telemetry.addData("Goal X", autoAimWithOdometry.getBackdropPoseX());
        telemetry.addData("Goal Y", autoAimWithOdometry.getBackdropPoseY());
//        telemetry.addData("Recovery Time", flywheel.getLastRecoveryTime());
        if (currentPose != null) {
            telemetry.addData("Robot X", currentPose.getX());
            telemetry.addData("Robot Y", currentPose.getY());
            telemetry.addData("Robot H", Math.toDegrees(currentPose.getHeading()));
        }
        telemetry.update();
    }
}
