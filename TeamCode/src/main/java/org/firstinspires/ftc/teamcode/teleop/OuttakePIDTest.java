package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DoubleMotorOuttakePID;

@TeleOp(name = "Outtake RPM PID Test")
public class OuttakePIDTest extends LinearOpMode {
    private DoubleMotorOuttakePID outtake;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new DoubleMotorOuttakePID(hardwareMap);

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData(">", "Connect to Panels dashboard");
        telemetry.addData(">", "Press A for target RPM, B to stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Button Logic ---
            if (gamepad1.a) {
                outtake.setTargetRPM(3500);  // Example target RPM for shooter
            } else if (gamepad1.b) {
                outtake.stop();
            }
            // --- PID Tuning ---
            else if (gamepad1.left_bumper) {
                outtake.increaseP();
            } else if (gamepad1.right_bumper) {
                outtake.decreaseP();
            } else if (gamepad1.dpad_right) {
                outtake.increaseI();
            } else if (gamepad1.dpad_left) {
                outtake.decreaseI();
            }
// --- D tuning (Up/Down D-Pad for D) ---
            else if (gamepad1.dpad_up) {
                outtake.increaseD();
            } else if (gamepad1.dpad_down) {
                outtake.decreaseD();
            }
// --- F tuning (X/Y buttons) ---
            else if (gamepad1.x) {
                outtake.increaseF();
            } else if (gamepad1.y) {
                outtake.decreaseF();
            }

            // --- PID Update ---
            outtake.update();

            panelsTelemetry.getTelemetry().addData("Target RPM", outtake.getTargetRPM());
            panelsTelemetry.getTelemetry().addData("Current RPM", outtake.getCurrentRPM());
            panelsTelemetry.getTelemetry().addData("kP", outtake.getP());
            panelsTelemetry.getTelemetry().addData("kI", outtake.getI());
            panelsTelemetry.getTelemetry().addData("kD", outtake.getD());
            panelsTelemetry.getTelemetry().addData("kF", outtake.getF());
            panelsTelemetry.getTelemetry().update();


            sleep(20);
        }
    }
}
