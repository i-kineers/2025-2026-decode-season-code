package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Chassis {
    // Declare our motors
    // Make sure your ID's match your configuration
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    public Chassis(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "flmotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "blmotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frmotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "brmotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runMacanumWheels(double leftStickX, double leftStickY, double rightStickX) {
        double y = leftStickY; // Remember, Y stick value is reversed
        double x = -leftStickX * 1.1; // Counteract imperfect strafing
        double rx = rightStickX;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}