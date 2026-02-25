package org.firstinspires.ftc.teamcode.SocalCode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This OpMode demonstrates how to use a distance sensor to detect
 * if an object (like a ball) is within a certain range.
 */
@TeleOp(name = "Distance Sensor Test")
public class DistanceSensorTest extends LinearOpMode {

    private DistanceSensor distanceSensor;

    // Adjust this value based on your robot's physical setup
    // This is the distance in centimeters to consider an object "detected"
    private static final double DETECTION_THRESHOLD_CM = 3.0;

    @Override
    public void runOpMode() {
        // Step 1: Initialize the distance sensor
        // Make sure to name the sensor "distance_sensor" in your robot's configuration.
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ds");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Step 2: Read the distance from the sensor
            double distance = distanceSensor.getDistance(DistanceUnit.CM);

            // Step 3: Check if the distance is within our detection threshold
            boolean ballDetected = distance < DETECTION_THRESHOLD_CM;

            // Step 4: Display the results on the Driver Station
            telemetry.addData("Ball Detected", ballDetected);
            telemetry.addData("Distance (cm)", "%.2f", distance);
            telemetry.update();
        }
    }
}
