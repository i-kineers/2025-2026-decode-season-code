package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;

@TeleOp(name = "Field Centric Drive Test", group = "Tests")
public class FieldCentricDriveTest extends LinearOpMode {

    private FieldCentricDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- INITIALIZATION --- 
        drive = new FieldCentricDrive(hardwareMap);

        // --- WAIT FOR START --- 
        telemetry.addLine("Ready to drive! Press the start button on the driver station.");
        telemetry.addLine("Press the A button to reset the IMU heading.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {
            // --- DRIVE ---
            drive.drive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            // --- IMU RESET ---
            if (gamepad1.a) {
                drive.resetIMU();
                telemetry.addLine("IMU Reset.");
                telemetry.update();
            }
        }
    }
}