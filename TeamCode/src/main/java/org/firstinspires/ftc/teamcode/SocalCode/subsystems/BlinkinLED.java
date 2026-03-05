package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BlinkinLED {
    private DistanceSensor distanceSensor;
    private DistanceSensor distanceSensor2;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    private static final double DETECTION_THRESHOLD_CM = 11.0;

    private double distance;
    private double distance2;

    private boolean ballDetected;
    private boolean ballDetected2;

    public BlinkinLED(HardwareMap hardwareMap){
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ds");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "ds2");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }
    public void runLED(){
        distance = distanceSensor.getDistance(DistanceUnit.CM);
        distance2 = distanceSensor2.getDistance(DistanceUnit.CM);

        ballDetected = distance < DETECTION_THRESHOLD_CM;
        ballDetected2 = distance2 < DETECTION_THRESHOLD_CM;

        if (ballDetected && ballDetected2) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN; // ALL BALLS DETECTED
            blinkinLedDriver.setPattern(pattern);

        } else if (ballDetected || ballDetected2) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
            blinkinLedDriver.setPattern(pattern);

        } else {
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriver.setPattern(pattern);
        }
    }

    public double sensorDistance() {
        return distance;
    }

    public boolean isBallDetected() {
        return ballDetected;
    }
}
