package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DoubleMotorOuttakePID;

@TeleOp(name = "Outtake RPM PID Test")
public class FShooterTest extends LinearOpMode {
    private DoubleMotorOuttakePID outtake;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    double targetRPM = 2000;

    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new DoubleMotorOuttakePID(hardwareMap);
        
        waitForStart();

        while (opModeIsActive()) {
            // --- Button Logic ---
            if (gamepad1.a) {
                outtake.setTargetRPM(targetRPM);  // Example target RPM for shooter
                outtake.stopLoader();
            } else if (gamepad1.b) {
                outtake.setTargetRPM(targetRPM);
                outtake.runLoader();
            } else if (gamepad1.x) {
                outtake.stop();
            }
            // --- Increase/Decrease target rpm ---
            else if (gamepad1.rightBumperWasPressed()) {
                targetRPM += 100;
            } else if (gamepad1.leftBumperWasPressed()) {
                targetRPM -= 100;
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

            telemetry.addData("Target RPM", outtake.getTargetRPM());
            telemetry.addData("Current RPM", outtake.getCurrentRPM());
            telemetry.addData("kP", outtake.getP());
            telemetry.addData("kI", outtake.getI());
            telemetry.addData("kD", outtake.getD());
            telemetry.addData("kF", outtake.getF());
            telemetry.update();

            sleep(20);
        }
    }
}
