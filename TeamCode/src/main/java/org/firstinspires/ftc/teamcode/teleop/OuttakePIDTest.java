package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DoubleMotorOuttakePID;

@TeleOp(name = "Outtake RPM PID Test")
public class OuttakePIDTest extends LinearOpMode {
    private DoubleMotorOuttakePID outtake;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    double targetRPM = 2000;

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
                outtake.setTargetRPM(targetRPM);  // Example target RPM for shooter
            } else if (gamepad1.b) {
                outtake.stop();
            }
            // --- Increase/Decrease target rpm ---
            else if (gamepad1.rightBumperWasPressed()) {
                targetRPM += 100;
            } else if (gamepad1.leftBumperWasPressed()) {
                targetRPM -= 100;
            }
            // --- P tuning (X/Y buttons) ---
            else if (gamepad1.xWasPressed()) {
                outtake.increaseP();
            } else if (gamepad1.yWasPressed()) {
                outtake.decreaseP();
            }
            // --- I tuning (Left/Right D-Pad for I) ---
            else if (gamepad1.dpadRightWasPressed()) {
                outtake.increaseI();
            } else if (gamepad1.dpadLeftWasPressed()) {
                outtake.decreaseI();
            }
            // --- D tuning (Up/Down D-Pad for D) ---
            else if (gamepad1.dpadUpWasPressed()) {
                outtake.increaseD();
            } else if (gamepad1.dpadDownWasPressed()) {
                outtake.decreaseD();
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
