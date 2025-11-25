package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;

import org.firstinspires.ftc.teamcode.subsystems.DoubleMotorOuttakePID;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;


@TeleOp(name="Main")
public class MainTeleop extends OpMode {

    private PanelsTelemetry panelsTelemetry;
    private Chassis chassis; // Old drive code
    private FieldCentricDrive drive;
    private DoubleMotorOuttakePID outtake;
    private Intake intake;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        chassis = new Chassis(hardwareMap); // Old drive code
        drive = new FieldCentricDrive(hardwareMap);
        outtake = new DoubleMotorOuttakePID(hardwareMap);
        intake = new Intake(hardwareMap);
    }



    @Override
    public void loop() {
        drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


        if (gamepad1.right_trigger == 1) {
            outtake.setTargetRPM(3000);
        } else if (gamepad1.right_trigger == 0) {
            outtake.setTargetRPM(0);
        }
//        else if (gamepad1.bWasPressed()) { // Also not needed if FSM isn't used
//            outtake.toggleRapidShoot();
//        }
        else if (gamepad1.right_bumper) {
            outtake.toggleServos(1);
            intake.runGate(1); // Open Gate
        } else if (!gamepad1.right_bumper) {
            outtake.toggleServos(0);
            intake.stopGate(); // Close Gate
        }
        else if (gamepad1.left_trigger == 1) {
            intake.runIntake(0.5);
        } else if (gamepad1.left_trigger == 0) {
            intake.stopIntake();
        } else if (gamepad1.aWasPressed()) {
            drive.resetIMU();
        }
//        else if (gamepad1.dpadUpWasPressed()) { // Becomes useless after removing FSM shooter
//            outtake.increaseTargetRPM();
//        } else if (gamepad1.dpadDownWasPressed()) {
//            outtake.decreaseTargetRPM();
//        }

        // Must call to run other functions in Outtake
//        outtake.startShooterLogic(); // Used for FSM shooter
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
