package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.OuttakePID;

@TeleOp(name = "Outtake RPM PID Test")
public class OuttakePIDTest extends LinearOpMode {
    private OuttakePID outtakePID;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    @Override
    public void runOpMode() throws InterruptedException {
        outtakePID = new OuttakePID(hardwareMap);

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData(">", "Connect to Panels dashboard");
        telemetry.addData(">", "Press A for target RPM, B to stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Button Logic ---
            if (gamepad1.a) {
                outtakePID.setTargetRPM(3500);  // Example target RPM for shooter
            } else if (gamepad1.b) {
                outtakePID.stop();
            }
            // --- PID Tuning ---
            else if (gamepad1.left_bumper) {
                outtakePID.increaseP();
            } else if (gamepad1.right_bumper) {
                outtakePID.decreaseP();
            } else if (gamepad1.dpad_right) {
                outtakePID.increaseI();
            } else if (gamepad1.dpad_left) {
                outtakePID.decreaseI();
            }
// --- D tuning (Up/Down D-Pad for D) ---
            else if (gamepad1.dpad_up) {
                outtakePID.increaseD();
            } else if (gamepad1.dpad_down) {
                outtakePID.decreaseD();
            }
// --- F tuning (X/Y buttons) ---
            else if (gamepad1.x) {
                outtakePID.increaseF();
            } else if (gamepad1.y) {
                outtakePID.decreaseF();
            }

            // --- PID Update ---
            outtakePID.update();

            panelsTelemetry.getTelemetry().addData("Target RPM", outtakePID.getTargetRPM());
            panelsTelemetry.getTelemetry().addData("Current RPM", outtakePID.getCurrentRPM());
            panelsTelemetry.getTelemetry().addData("kP", outtakePID.getP());
            panelsTelemetry.getTelemetry().addData("kI", outtakePID.getI());
            panelsTelemetry.getTelemetry().addData("kD", outtakePID.getD());
            panelsTelemetry.getTelemetry().addData("kF", outtakePID.getF());
            panelsTelemetry.getTelemetry().update();


            sleep(20);
        }
    }
}
