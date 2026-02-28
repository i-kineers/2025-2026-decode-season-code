package org.firstinspires.ftc.teamcode.SocalCode.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Distance Sensor with led Test")
public class DistanceSensorWLed extends LinearOpMode {
    private DistanceSensor distanceSensor;
    private DistanceSensor distanceSensor2;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    // Adjust this value based on your robot's physical setup
    // This is the distance in centimeters to consider an object "detected"
    private static final double DETECTION_THRESHOLD_CM = 5.0;
    private static final double TargetRPM = 3500;
    private int Counter= 0;
    public int getCurrentRPM(){
        Counter+=100;
        if(Counter>=5000){
            Counter = 0;
        }
        return Counter;
    }

    @Override
    public void runOpMode() {
        // Step 1: Initialize the distance sensor
        // Make sure to name the sensor "distance_sensor" in your robot's configuration.
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ds");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "ds2");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Step 2: Read the distance from the sensor
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            double distance2 = distanceSensor2.getDistance(DistanceUnit.CM);

            // Step 3: Check if the distance is within our detection threshold
            boolean ballDetected = distance < DETECTION_THRESHOLD_CM;
            boolean ballDetected2 = distance2 < DETECTION_THRESHOLD_CM;

            int CurRPM = getCurrentRPM();
            sleep(500);

            if (ballDetected && ballDetected2 && CurRPM>=TargetRPM) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
                blinkinLedDriver.setPattern(pattern);

            } else if (ballDetected && ballDetected2 && CurRPM < TargetRPM){
                pattern = RevBlinkinLedDriver.BlinkinPattern.LIME;
                blinkinLedDriver.setPattern(pattern);

            } else if ((ballDetected||ballDetected2) && CurRPM>=TargetRPM) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                blinkinLedDriver.setPattern(pattern);

            } else if ((ballDetected2 || ballDetected) && CurRPM<TargetRPM){
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                blinkinLedDriver.setPattern(pattern);

            } else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;
                blinkinLedDriver.setPattern(pattern);
            }


            // Step 4: Display the results on the Driver Station
            telemetry.addData("Ball Detected", ballDetected);
            telemetry.addData("Distance (cm)", "%.2f", distance);
            telemetry.addData("Ball Detected", ballDetected2);
            telemetry.addData("Distance (cm)", "%.2f", distance2);
            telemetry.update();
        }
    }
}
