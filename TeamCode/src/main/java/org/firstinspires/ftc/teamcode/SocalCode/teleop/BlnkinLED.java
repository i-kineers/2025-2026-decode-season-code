package org.firstinspires.ftc.teamcode.SocalCode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.BlinkinLED;
@TeleOp(name = "Distance Sensor With Blinkin")
public class BlnkinLED extends OpMode {
    private BlinkinLED blinkinLED;

    public void init() {
        blinkinLED = new BlinkinLED(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    @Override
    public void loop() {
        blinkinLED.runLED();
    }
}
