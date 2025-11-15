package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;

import org.firstinspires.ftc.teamcode.subsystems.DoubleMotorOuttakePID;


@TeleOp(name="Main")
public class MainTeleop extends OpMode {

    private PanelsTelemetry panelsTelemetry;
    private Chassis chassis;
    private DoubleMotorOuttakePID outtake;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        chassis = new Chassis(hardwareMap);
        outtake = new DoubleMotorOuttakePID(hardwareMap);
    }



    @Override
    public void loop() {
        chassis.runMacanumWheels(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (gamepad1.aWasPressed()) {
            outtake.nextState();
        } else if (gamepad1.bWasPressed()) {
            outtake.toggleRapidShoot();
        } else if (gamepad1.left_trigger == 1) {
            outtake.toggleServos(1);
        } else if (gamepad1.left_trigger == 0) {
            outtake.toggleServos(0);
        } else if (gamepad1.dpadUpWasPressed()) {
            outtake.increaseTargetRPM();
        } else if (gamepad1.dpadDownWasPressed()) {
            outtake.decreaseTargetRPM();
        }

        // Must call to run other functions in Outtake
        outtake.startShooterLogic();
        outtake.update();

        // --- Panels Telemetry ---
        panelsTelemetry.getTelemetry().addData("Target RPM", outtake.getTargetRPM());
        panelsTelemetry.getTelemetry().addData("Current RPM", outtake.getCurrentRPM());
        panelsTelemetry.getTelemetry().addData("kP", outtake.getP());
        panelsTelemetry.getTelemetry().addData("kI", outtake.getI());
        panelsTelemetry.getTelemetry().addData("kD", outtake.getD());
        panelsTelemetry.getTelemetry().addData("kF", outtake.getF());
        panelsTelemetry.getTelemetry().update();

        // --- Standard Driver Hub Telemetry ---
        telemetry.addData("Target RPM", outtake.getTargetRPM());
        telemetry.addData("Current RPM", outtake.getCurrentRPM());
        telemetry.addData("kP", outtake.getP());
        telemetry.addData("kI", outtake.getI());
        telemetry.addData("kD", outtake.getD());
        telemetry.addData("kF", outtake.getF());
        telemetry.addData("Current State", outtake.getState());
        telemetry.addData("Toggled Rapid Shooter", outtake.getRapidShooterState());
        telemetry.addData("Toggled Servos", outtake.getServoState());
        telemetry.update();

        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
