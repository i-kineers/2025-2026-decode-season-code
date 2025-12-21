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

    private boolean rightTriggerPressed = false;
    private boolean rightBumperPressed = false;
    private boolean leftTriggerPressed = false;
    private boolean leftBumperPressed = false;

    private double targetRPM = 3000;

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
        // --- 1. Drive Control (Always checked) ---
        drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        chassis.runMacanumWheels(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        // --- 2. Outtake/Shooter Controls (Independent IF blocks) ---

        // A. Right Trigger (Spin up/down)
        if (gamepad1.right_trigger > 0.5) { // Use a threshold > 0 for analog triggers
            outtake.setTargetRPM(targetRPM);
            rightTriggerPressed = true;
        } else if (gamepad1.right_trigger < 0.1) {
            outtake.stop();
            rightTriggerPressed = false;
        }

        // B. Right Bumper (Feed/Gate) - Use independent checks
        if (gamepad1.right_bumper) {
            outtake.runLoader();
//            intake.runGate(0); // Open Gate
            rightBumperPressed = true;
        } else { // This else should only apply to the Right Bumper's functions
            outtake.stopLoader();
//            intake.runGate(1); // Close Gate
            rightBumperPressed = false;
        }


        // --- 3. Intake Controls (Independent IF blocks) ---

        // C. Left Trigger (Intake Reverse)
        if (gamepad1.left_trigger > 0.5) { // Use a threshold > 0 for analog triggers
            intake.runIntake(-1);
            intake.runGate(0.75);
            leftTriggerPressed = true;
        } else if (gamepad1.left_trigger < 0.1) {
            intake.stopIntake();
            intake.runGate(0);
            leftTriggerPressed = false;
        }

        // D. Left Bumper (Intake Forward)
        if (gamepad1.left_bumper) {
            intake.runIntake(1);
            leftBumperPressed = true;
        } else { // This else should only apply to the Left Bumper's function
            leftBumperPressed = false;
        }

        if (gamepad1.dpadUpWasPressed()) {
            targetRPM += 100;
        } else if (gamepad1.dpadDownWasPressed()) {
            targetRPM -= 100;
        }

        // F. Reset IMU (Toggle Button)
        if (gamepad1.a) { // Check for a *press* event
            drive.resetIMU();
            telemetry.addLine("IMU Reset.");
        }


        // --- Remaining Logic (Always checked) ---
        outtake.update();
        // ... Telemetry and Thread.sleep(20)

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
        telemetry.addData("Current State", outtake.getState());
        telemetry.addData("Toggled Rapid Shooter", outtake.getRapidShooterState());
        telemetry.addData("Toggled Servos", outtake.getServoState());
        telemetry.addData("rightTriggerPressed", rightTriggerPressed);
        telemetry.addData("rightBumperPressed", rightBumperPressed);
        telemetry.addData("leftTriggerPressed", leftTriggerPressed);
        telemetry.addData("leftBumperPressed", leftBumperPressed);
        telemetry.update();

        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
