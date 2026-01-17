package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSystem;

@TeleOp(name = "Flywheel TeleOp (Panels + PIDF Tuning)")
public class FlywheelTeleOp extends LinearOpMode {

    private FlywheelSystem flywheel;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    private double targetTPS;

    @Override
    public void runOpMode() {

        flywheel = new FlywheelSystem(hardwareMap);
        targetTPS = flywheel.LOW_TARGET_TPS;

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData(">", "Connect to Panels dashboard");
        telemetry.addData(">", "DPad Up/Down = Target TPS");
        telemetry.addData(">", "RB = Start | LB = Stop");
        telemetry.addData(">", "X/Y = P +/- | DPad L/R = I +/- | DPad U/D = D +/-");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /* ===================== INPUT ===================== */

            // --- Target selection ---
            if (gamepad1.dpadUpWasPressed()) {
                targetTPS = flywheel.HIGH_TARGET_TPS;
                flywheel.setTargetTPS(targetTPS);
            } else if (gamepad1.dpadDownWasPressed()) {
                targetTPS = flywheel.LOW_TARGET_TPS;
                flywheel.setTargetTPS(targetTPS);
            }

            // --- Firing control ---
            if (gamepad1.rightBumperWasPressed()) {
                targetTPS = 0;
                flywheel.setTargetTPS(targetTPS);
            }

            // --- PIDF Tuning ---
            // P tuning
            if (gamepad1.xWasPressed()) {
                flywheel.kP += 0.0001;
            } else if (gamepad1.yWasPressed()) {
                flywheel.kP = Math.max(0, flywheel.kP - 0.0001);
            }

            // I tuning (Left/Right DPad)
            if (gamepad1.dpadRightWasPressed()) {
                flywheel.kI += 0.00001;
            } else if (gamepad1.dpadLeftWasPressed()) {
                flywheel.kI = Math.max(0, flywheel.kI - 0.00001);
            }

            // D tuning (Up/Down DPad)
            if (gamepad1.dpadUpWasPressed()) {
                flywheel.kD += 0.0001;
            } else if (gamepad1.dpadDownWasPressed()) {
                flywheel.kD = Math.max(0, flywheel.kD - 0.0001);
            }

            // F tuning (bumpers)
            if (gamepad1.right_trigger > 0.5) {
                flywheel.kF_HIGH += 0.00001;
            } else if (gamepad1.left_trigger > 0.5) {
                flywheel.kF_HIGH = Math.max(0, flywheel.kF_HIGH - 0.00001);
            }

            /* ===================== UPDATE ===================== */
            flywheel.update();

            /* ===================== PANELS TELEMETRY ===================== */
            panelsTelemetry.getTelemetry().addData("Target TPS", targetTPS);
            panelsTelemetry.getTelemetry().addData("Current TPS", flywheel.getVelocity());
            panelsTelemetry.getTelemetry().addData("Shot State", flywheel.getShotState());
            panelsTelemetry.getTelemetry().addData("Last Recovery (ms)", flywheel.getLastRecoveryTime());
            panelsTelemetry.getTelemetry().addData("Shots Fired", flywheel.getRecoveryLog().size());
            panelsTelemetry.getTelemetry().addData("Danger Threshold", flywheel.DANGER_THRESHOLD);

            // PIDF values
            panelsTelemetry.getTelemetry().addData("kP", flywheel.kP);
            panelsTelemetry.getTelemetry().addData("kI", flywheel.kI);
            panelsTelemetry.getTelemetry().addData("kD", flywheel.kD);
            panelsTelemetry.getTelemetry().addData("kF_HIGH", flywheel.kF_HIGH);

            panelsTelemetry.getTelemetry().update();
        }
    }
}
