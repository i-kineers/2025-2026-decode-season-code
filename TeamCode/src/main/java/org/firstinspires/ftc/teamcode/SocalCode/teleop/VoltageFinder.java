package org.firstinspires.ftc.teamcode.SocalCode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "Voltage Finder")
public class VoltageFinder extends OpMode {

    private VoltageSensor batteryVoltageSensor;
    public double FeedForward;
    public double Voltage;

    @Override
    public void init() {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        double voltage = batteryVoltageSensor.getVoltage();
        telemetry.addData("Battery Voltage", "%.2f V", voltage);
        telemetry.update();

    }
    public void getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double Voltage = sensor.getVoltage();
        }
    }

    public void FindFeedForwardUsingVoltage() {
        if (gamepad1.aWasPressed()) {
            FeedForward = -1.76 * Voltage + 36.9;
        }
    }


}



// Add this method inside your OpMode class

