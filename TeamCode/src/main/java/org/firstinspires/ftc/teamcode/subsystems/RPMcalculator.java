package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx; // Use DcMotorEx for velocity methods
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RPMcalculator {

    // --- Constants ---
    // Found on the motor's datasheet (e.g., REV HD Hex, Gobilda Yellow Jacket)
    public static final double TICKS_PER_MOTOR_REV = 537.7;

    // --- Hardware ---
    private final DcMotorEx launcherMotor;

    // --- Internal Calculation Variables ---
    private double lastPositionTicks = 0;
    private final ElapsedTime timer;
    private double lastCalculatedRPM = 0;

    /**
     * Constructor for the RPM Calculator subsystem.
     * @param hardwareMap The hardware map from your OpMode.
     */
    public RPMcalculator(HardwareMap hardwareMap) {
        // We get the motor here to read its position.
        // It's crucial to cast to DcMotorEx to use getVelocity().
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher"); // Use the name from your robot config
        timer = new ElapsedTime();
        timer.reset();
    }

    /**
     * This method MUST be called in every loop of your OpMode.
     * It reads the motor's position and updates the RPM calculation.
     */
    public void update() {
        double currentTime = timer.seconds();
        double currentPosition = launcherMotor.getCurrentPosition();

        // Calculate the change in time and position
        double deltaTime = currentTime; // timer is reset in the previous update
        double deltaTicks = currentPosition - lastPositionTicks;

        // Calculate RPM based on the changes
        // Failsafe to prevent division by zero
        if (deltaTime > 0) {
            double revolutions = deltaTicks / TICKS_PER_MOTOR_REV;
            lastCalculatedRPM = (revolutions / deltaTime) * 60.0; // Revolutions per minute
        }

        // Reset for the next calculation cycle
        lastPositionTicks = currentPosition;
        timer.reset();
    }

    /**
     * A simpler, more direct way to get RPM using the FTC SDK's built-in velocity calculation.
     * This is often more stable than a manual calculation.
     */
    public void updateUsingSDK() {
        // getVelocity() returns ticks per second.
        double ticksPerSecond = launcherMotor.getVelocity();
        // Convert ticks per second to revolutions per minute
        lastCalculatedRPM = (ticksPerSecond / TICKS_PER_MOTOR_REV) * 60.0;
    }


    /**
     * @return The last calculated RPM value.
     */
    public double getRPM() {
        return lastCalculatedRPM;
    }
}
