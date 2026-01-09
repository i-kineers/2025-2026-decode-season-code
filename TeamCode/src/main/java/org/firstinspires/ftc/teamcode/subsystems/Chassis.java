package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Chassis {
    // Motor declarations
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    // Ramping instances for each axis of movement
    private final InputRamper forwardRamper = new InputRamper();
    private final InputRamper strafeRamper = new InputRamper();
    private final InputRamper turnRamper = new InputRamper();

    public Chassis(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "flmotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "blmotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frmotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "brmotor");

        // Reverse left side so positive power moves robot forward
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set to BRAKE to prevent sliding after stick release
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runMacanumWheels(double leftStickX, double leftStickY, double rightStickX) {
        // 1. Apply Ramping to inputs to reduce jerkiness
        double rampedY = forwardRamper.rampInput(leftStickY);
        double rampedX = strafeRamper.rampInput(leftStickX);
        double rampedRX = turnRamper.rampInput(rightStickX);

        // 2. Calculate Mecanum Kinematics
        double y = -rampedY; // Y stick is reversed
        double x = rampedX * 1.1; // Counteract imperfect strafing
        double rx = rampedRX * 0.4; // Scaled down rotation for control

        // 3. Normalize powers so no motor exceeds 1.0
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // 4. Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Internal class to handle smooth acceleration
     */
    private static class InputRamper {
        private final double MIN_MULTIPLIER = 0.5;
        private final double MAX_MULTIPLIER = 1.00;
        private final double incrementMultiplier = 0.20;
        private final double timeIncrementInMs = 150; // Slightly faster ramp for better feel

        private final ElapsedTime rampTimer = new ElapsedTime();
        private double multiplier = MIN_MULTIPLIER;

        public double rampInput(double input) {
            if (input != 0) {
                if (rampTimer.milliseconds() > timeIncrementInMs) {
                    multiplier += incrementMultiplier;
                    rampTimer.reset();
                }
            } else {
                multiplier = MIN_MULTIPLIER;
                rampTimer.reset();
            }

            multiplier = Math.max(MIN_MULTIPLIER, Math.min(multiplier, MAX_MULTIPLIER));
            return input * multiplier;
        }
    }
}