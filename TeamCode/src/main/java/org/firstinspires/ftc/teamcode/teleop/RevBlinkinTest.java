package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "REV Blinkin LED Test", group = "Test")
public class RevBlinkinTest extends OpMode {

    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void init() {
        // Get the Blinkin LED Driver from the hardware map
        // Make sure to configure it as "blinkin" in the Robot Configuration
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        // Set an initial pattern (e.g., RED)
        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriver.setPattern(pattern);
    }

    @Override
    public void loop() {
        // Example: Change pattern based on gamepad input

        if (gamepad1.a) {
            // Solid Green
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if (gamepad1.b) {
            // Solid Red
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        } else if (gamepad1.x) {
            // Blue
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        } else if (gamepad1.y) {
            // Gold
            pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
        } else if (gamepad1.dpad_up) {
            // Rainbow with glittering
            pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;
        } else if (gamepad1.dpad_down) {
            // Heartbeat Red
            pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
        }

        // Update the pattern
        blinkinLedDriver.setPattern(pattern);

        telemetry.addData("Current Pattern", pattern.toString());
        telemetry.update();
    }
}
