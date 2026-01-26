package org.firstinspires.ftc.teamcode.CodePriorILT.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class LEDControl {
    private LED redLed;
    private LED greenLed;

    public void init(HardwareMap hardwareMap) {
        redLed = hardwareMap.get(LED.class, "red_led");
        greenLed = hardwareMap.get(LED.class, "green_led");
    }

    public void setRedLed(boolean isOn) {
        if (isOn) {
            redLed.on();
        }
        else {
            redLed.off();
        }

    }

    public void setGreenLed(boolean isOn) {
        if (isOn) {
            greenLed.on();
        }
        else {
            greenLed.off();
        }
    }
}
