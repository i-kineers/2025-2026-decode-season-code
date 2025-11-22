package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This is a simple teleop program for testing a mecanum drive chassis.
 * It uses robot-centric controls.
 */
@TeleOp(name = "Mecanum Drive Test", group = "Tests")
public class MecanumDriveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // --- HARDWARE MAPPING ---
        // Declare our motors. Make sure your ID's match your configuration.
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("flmotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("blmotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frmotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("brmotor");

        // --- MOTOR DIRECTIONS ---
        // Reverse the motors on the left side, based on your other files.
        // If your robot spins instead of driving forward, reverse the right side instead.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- WAIT FOR START ---
        telemetry.addLine("Ready to drive!");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {
            // --- MECANUM DRIVE LOGIC ---
            // This is "robot-centric" drive. The controls are relative to the robot's front.
            double y = -gamepad1.left_stick_y; // Forward/Backward
            double x = gamepad1.left_stick_x * 1.1; // Strafe Left/Right (with correction)
            double rx = gamepad1.right_stick_x * 0.7; // Rotate Left/Right (with reduced speed)

            // --- WHEEL POWER CALCULATIONS ---
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all powers maintain the same ratio without exceeding [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // --- SET MOTOR POWERS ---
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // --- TELEMETRY --- (Optional)
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.update();
        }
    }
}
