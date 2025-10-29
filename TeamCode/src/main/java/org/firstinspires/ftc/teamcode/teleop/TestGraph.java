package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.*;

@TeleOp(name = "Test Graph")
public class TestGraph extends OpMode {
    private PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private ElapsedTime timer = new ElapsedTime();

    private double sinVariable = 0.0;
    private double cosVariable = 0.0;
    private double constVariable = 0.0;
    private double dampedSine = 0.0;
    private double lissajous = 0.0;
    private double ramp = 0.0;
    private double squareWave = 0.0;

    @Override
    public void init() {
        timer.reset();
        updateSignals();
    }

    @Override
    public void loop() {
        updateSignals();
    }

    private void updateSignals() {
        double t = timer.seconds();
        sinVariable = sin(t);
        cosVariable = cos(t);
        constVariable = 1.0;
        dampedSine = exp(-0.2 * t) * sin(2 * t);
        lissajous = sin(3 * t + Math.PI / 2) * cos(2 * t);
        ramp = (t % 5.0) / 5.0;
        squareWave = (sin(2 * Math.PI * 0.5 * t) > 0) ? 1.0 : -1.0;

        panelsTelemetry.getTelemetry().addData("sin", sinVariable);
        panelsTelemetry.getTelemetry().addData("cos", cosVariable);
        panelsTelemetry.getTelemetry().addData("dampedSine", dampedSine);
        panelsTelemetry.getTelemetry().addData("lissajous", lissajous);
        panelsTelemetry.getTelemetry().addData("ramp", ramp);
        panelsTelemetry.getTelemetry().addData("square", squareWave);
        panelsTelemetry.getTelemetry().addData("const", constVariable);

        panelsTelemetry.getTelemetry().addData("time", t);
        panelsTelemetry.getTelemetry().addData("time_squared", t * t);

        panelsTelemetry.getTelemetry().update(telemetry);
    }
}
