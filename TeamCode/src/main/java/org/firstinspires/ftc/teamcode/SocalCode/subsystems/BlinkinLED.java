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
    public BlinkinLED(HardwareMap hardwareMap){
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ds");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "ds2");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }
    public void runLED(){
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        double distance2 = distanceSensor2.getDistance(DistanceUnit.CM);

        // Step 3: Check if the distance is within our detection threshold
        boolean ballDetected = distance < DETECTION_THRESHOLD_CM;
        boolean ballDetected2 = distance2 < DETECTION_THRESHOLD_CM;

        int CurRPM = getCurrentRPM();

        if (ballDetected && ballDetected2 && CurRPM>=TargetRPM) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
            blinkinLedDriver.setPattern(pattern);

        } else if (ballDetected && ballDetected2 && CurRPM < TargetRPM){
            pattern = RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE;
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
    }
}
