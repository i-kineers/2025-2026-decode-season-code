package org.firstinspires.ftc.teamcode.SocalCode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.FlywheelSystem;

@TeleOp(name = "Flywheel TeleOp (Panels + PIDF Tuning)")
@Disabled
public class FlywheelTeleOp extends LinearOpMode {

    private FlywheelSystem flywheel;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    private double targetTPS;

    @Override
    public void runOpMode() {

        flywheel = new FlywheelSystem(hardwareMap);
        targetTPS = flywheel.LOW_TARGET_TPS;
        flywheel.setTargetTPS(targetTPS);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) {
                targetTPS += 50;
                flywheel.setTargetTPS(targetTPS);
            }
            else if (gamepad1.dpadDownWasPressed()) {
                targetTPS = Math.max(0, targetTPS - 50);
                flywheel.setTargetTPS(targetTPS);
            }

            if (gamepad1.rightBumperWasPressed()) {
                targetTPS = 0;
                flywheel.setTargetTPS(0);
            }

            if (gamepad1.xWasPressed()) {
                flywheel.kP += 0.0001;
            }
            else if (gamepad1.bWasPressed()) {
                flywheel.kP = Math.max(0, flywheel.kP - 0.0001);
            }

            if (gamepad1.dpadRightWasPressed()) {
                flywheel.kI += 0.00001;
            }
            else if (gamepad1.dpadLeftWasPressed()) {
                flywheel.kI = Math.max(0, flywheel.kI - 0.00001);
            }

            if (gamepad1.leftBumperWasPressed()) {
                flywheel.kD += 0.0001;
            }

            if (gamepad1.startWasPressed()) {
                flywheel.kD = Math.max(0, flywheel.kD - 0.0001);
            }

            flywheel.cycleShootingState(gamepad1);
            flywheel.update(gamepad1);

            panelsTelemetry.getTelemetry().addData("Target TPS", targetTPS);
            panelsTelemetry.getTelemetry().addData("Current TPS", flywheel.getVelocity());
            panelsTelemetry.getTelemetry().addData("Shooter State", flywheel.getShotState());
            panelsTelemetry.getTelemetry().addData("Last Recovery (ms)", flywheel.getLastRecoveryTime());
            panelsTelemetry.getTelemetry().addData("kP", flywheel.kP);
            panelsTelemetry.getTelemetry().addData("kI", flywheel.kI);
            panelsTelemetry.getTelemetry().addData("kD", flywheel.kD);
            panelsTelemetry.getTelemetry().addData("kF_HIGH", flywheel.kF_HIGH);
            panelsTelemetry.getTelemetry().update();
        }
    }
}