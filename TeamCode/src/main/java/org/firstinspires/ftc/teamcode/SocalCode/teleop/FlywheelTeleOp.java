package org.firstinspires.ftc.teamcode.SocalCode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.DoubleIntake;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.FlywheelSystem;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.Parking;

@TeleOp(name = "Flywheel TeleOp (Panels + PIDF Tuning)")
public class FlywheelTeleOp extends LinearOpMode {

    private FlywheelSystem flywheel;
    private DoubleIntake intake;
    private Parking parking;
    private FieldCentricDrive fieldCentricDrive;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    boolean stop = false;

    private double targetTPS;
    private double LOW_TARGET_TPS;
    private double HIGH_TARGET_TPS;

    @Override
    public void runOpMode() {

        flywheel = new FlywheelSystem(hardwareMap);
        intake = new DoubleIntake(hardwareMap);
        parking = new Parking(hardwareMap);
        fieldCentricDrive = new FieldCentricDrive(hardwareMap);
        targetTPS = flywheel.LOW_TARGET_TPS;
        LOW_TARGET_TPS = 1213;
        HIGH_TARGET_TPS = 1500;
        flywheel.setTargetTPS(targetTPS);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) {
                targetTPS += 10;
                flywheel.setTargetTPS(targetTPS);
            }
            else if (gamepad1.dpadDownWasPressed()) {
                targetTPS = Math.max(0, targetTPS - 10);
                flywheel.setTargetTPS(targetTPS);
            }

            if (gamepad1.aWasPressed()) {
                targetTPS = LOW_TARGET_TPS;
                flywheel.setTargetTPS(targetTPS);
            }
            else if (gamepad1.bWasPressed()) {
                targetTPS = HIGH_TARGET_TPS;
                flywheel.setTargetTPS(targetTPS);
            }

            if (gamepad1.rightBumperWasPressed()) {
                flywheel.setTargetTPS(0);
//                if (!stop) { stop = true; }
//                else { stop = false; }
            }

//            if (gamepad1.xWasPressed()) {
//                flywheel.kP += 0.0001;
//            }
//            else if (gamepad1.bWasPressed()) {
//                flywheel.kP = Math.max(0, flywheel.kP - 0.0001);
//            }

            if (gamepad1.dpadRightWasPressed()) {
                flywheel.kP += 0.0001;
//                flywheel.kI += 0.00001;
            }
            else if (gamepad1.dpadLeftWasPressed()) {
                flywheel.kP = Math.max(0, flywheel.kP - 0.0001);
//                flywheel.kI = Math.max(0, flywheel.kI - 0.00001);
            }

//            if (gamepad1.leftBumperWasPressed()) {
//                flywheel.kD += 0.0001;
//            }
//
//            if (gamepad1.startWasPressed()) {
//                flywheel.kD = Math.max(0, flywheel.kD - 0.0001);
//            }

            fieldCentricDrive.update(gamepad1);

            intake.runIntake(gamepad1);
            parking.update(gamepad1);

//            if (!stop) {
//            flywheel.cycleShootingState(gamepad1);
            flywheel.update(gamepad1);
//            } else if (stop) {
//                flywheel.setFlywheelPower(0);
//            }

            panelsTelemetry.getTelemetry().addData("Target TPS", targetTPS);
            panelsTelemetry.getTelemetry().addData("Current TPS", flywheel.getVelocity());
            panelsTelemetry.getTelemetry().addData("Shooter State", flywheel.getShotState());
            panelsTelemetry.getTelemetry().addData("Last Recovery (ms)", flywheel.getLastRecoveryTime());
            panelsTelemetry.getTelemetry().addData("kP", flywheel.kP);
            panelsTelemetry.getTelemetry().addData("stop", stop);
            panelsTelemetry.getTelemetry().addData("kI", flywheel.kI);
            panelsTelemetry.getTelemetry().addData("kD", flywheel.kD);
            panelsTelemetry.getTelemetry().addData("kF_HIGH", flywheel.kF_HIGH);
            panelsTelemetry.getTelemetry().update();
        }
    }
}