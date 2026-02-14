package org.firstinspires.ftc.teamcode.CodePriorILT.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CodePriorILT.subsystems.LEDControl;

@TeleOp(name = "REV Blinkin LED Test", group = "Test")
public class RevBlinkinTest extends OpMode {

   LEDControl LED = new LEDControl();


    @Override
    public void init() {
        LED.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            LED.setRedLed(true);
            LED.setGreenLed(false);
        }
        else if (gamepad1.b) {
            LED.setGreenLed(true);
            LED.setRedLed(false);
        }
        else if (gamepad1.y) {
            LED.setGreenLed(true);
            LED.setRedLed(true);
        }
        else if (gamepad1.x) {
            LED.setGreenLed(false);
            LED.setRedLed(false);
        }
    }
}
