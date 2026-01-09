package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldCentricDrive {
    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    private final IMU imu;

    public FieldCentricDrive(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "flmotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "blmotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frmotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "brmotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    public void drive(double leftStickY, double leftStickX, double rightStickX) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        
        // Call the static calculation method
        double[] powers = calculatePowers(leftStickY, leftStickX, rightStickX, heading);
        
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
        double rx = rightStickX * 0.4;

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
}
