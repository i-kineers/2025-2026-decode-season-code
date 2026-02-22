package org.firstinspires.ftc.teamcode.SocalCode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SocalCode.subsystems.Parking;

@TeleOp(name = "Parking TeleOp", group = "Test")
public class ParkingTeleop extends LinearOpMode {

    private Parking parking;

    @Override
    public void runOpMode() {

        // Initialize subsystem
        parking = new Parking(hardwareMap);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Update subsystem
            parking.update(gamepad1);

            // Telemetry
            telemetry.addData("Is Deployed (Park Mode)", parking.isDeployed());
            telemetry.addData("Drive Position", parking.getDrivePosition());
            telemetry.addData("Park Position", parking.getParkPosition());
            telemetry.update();
        }
    }
}