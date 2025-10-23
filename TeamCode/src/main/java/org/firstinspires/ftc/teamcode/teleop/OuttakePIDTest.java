package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.OuttakePID;

@TeleOp(name = "Outtake PID Test") // It's good practice to name your TeleOp
public class OuttakePIDTest extends LinearOpMode {
    private OuttakePID outtakePID;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the subsystem
        outtakePID = new OuttakePID(hardwareMap);

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData(">", "Press A to go to MAX position");
        telemetry.addData(">", "Press B to go to MIN position");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Control Logic ---
            // When gamepad 'A' is pressed, set the target to the MAX position
            if (gamepad1.a) {
                outtakePID.setTargetPosition(OuttakePID.LAUNCHER_MAX_POS);
            }
            // When gamepad 'B' is pressed, set the target to the MIN position
            else if (gamepad1.b) {
                outtakePID.setTargetPosition(OuttakePID.LAUNCHER_MIN_POS);
            }

            // --- THIS IS THE MOST IMPORTANT STEP ---
            // You must call update() in every loop.
            // This runs the PID calculations and sets the motor power.
            outtakePID.update();


            // --- Telemetry for Debugging ---
            // Display what the PID controller is doing
            telemetry.addData("Target Position", outtakePID.getTargetPosition());
            telemetry.addData("Current Position", outtakePID.getCurrentPosition());
            telemetry.addData("Calculated Power", outtakePID.getCalculatedPower());
            telemetry.update();
        }
    }
}
