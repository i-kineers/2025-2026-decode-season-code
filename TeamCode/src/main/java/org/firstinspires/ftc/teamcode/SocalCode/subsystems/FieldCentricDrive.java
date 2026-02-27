package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldCentricDrive {
    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    private final IMU imu;

    private final InputRamper yRamper;
    private final InputRamper xRamper;
    private final InputRamper rxRamper;

    public FieldCentricDrive(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "flmotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "blmotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frmotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "brmotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        yRamper = new InputRamper();
        xRamper = new InputRamper();
        rxRamper = new InputRamper();
    }

    /**
     * Handles all teleop drive logic from a gamepad, including IMU reset.
     * This should be called in the main loop.
     * @param gamepad The gamepad to read inputs from.
     */
    public void update(Gamepad gamepad) {
        // Standard drive controls, with turning inverted for intuitive control
        drive(gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);

        // Handle IMU reset on 'A' button press
        if (gamepad.a) {
            resetIMU();
        }
    }

    /**
     * Main drive method for use with custom inputs (like auto-aim).
     */
    public void drive(double leftStickY, double leftStickX, double rightStickX) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Ramp joystick inputs
        double rampedLeftStickY = yRamper.rampInput(leftStickY);
        double rampedLeftStickX = xRamper.rampInput(leftStickX);
        double rampedRightStickX = rxRamper.rampInput(rightStickX);

        // Call the static calculation method
        double[] powers = calculatePowers(rampedLeftStickY, rampedLeftStickX, rampedRightStickX, heading);

        frontLeftMotor.setPower(powers[0]);
        backLeftMotor.setPower(powers[1]);
        frontRightMotor.setPower(powers[2]);
        backRightMotor.setPower(powers[3]);
    }

    /**
     * STATIC mathematical calculation of motor powers.
     * This can be run on a PC without any robot hardware connected.
     * Returns array: [FL, BL, FR, BR]
     */
    public static double[] calculatePowers(double leftStickY, double leftStickX, double rightStickX, double botHeading) {
        double y = -leftStickY;
        double x = leftStickX;
        double rx = rightStickX;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        return new double[]{
            (rotY + rotX + rx) / denominator,
            (rotY - rotX + rx) / denominator,
            (rotY - rotX - rx) / denominator,
            (rotY + rotX - rx) / denominator
        };
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    /**
     * Inner class to handle joystick input ramping for smoother acceleration and deceleration.
     */
    private static class InputRamper {
        private final double MAX_DELTA_PER_SEC = 4.0;
        private final ElapsedTime timer = new ElapsedTime();

        private double currentOutput = 0.0;
        private final double DEADBAND = 0.05;

        public double rampInput(double input) {
            double target = Math.abs(input) > DEADBAND ? input : 0.0;

            double dt = timer.seconds();
            timer.reset();

            double maxDelta = MAX_DELTA_PER_SEC * dt;
            double delta = target - currentOutput;

            if (Math.abs(delta) > maxDelta) {
                delta = Math.signum(delta) * maxDelta;
            }

            currentOutput += delta;
            return currentOutput;
        }

        public void reset() {
            currentOutput = 0.0;
            timer.reset();
        }
    }
}
