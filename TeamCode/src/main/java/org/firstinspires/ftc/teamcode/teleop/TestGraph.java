package org.firstinspires.ftc.teamcode.graph;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.*;

@TeleOp(name = "Test Graph")
public class TestGraph extends OpMode {
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private final ElapsedTime timer = new ElapsedTime();

    private double sinVariable;
    private double cosVariable;
    private double constVariable;
    private double dampedSine;
    private double lissajous;
    private double ramp;
    private double squareWave;

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
        lissajous = sin(3 * t + PI / 2) * cos(2 * t);
        ramp = (t % 5.0) / 5.0;
        squareWave = sin(2 * PI * 0.5 * t) > 0 ? 1.0 : -1.0;

        // Send data to Panels dashboard
        panelsTelemetry.getTelemetry().addData("sin", sinVariable);
        panelsTelemetry.addData("sin", sinVariable);
        panelsTelemetry.addData("cos", cosVariable);
        panelsTelemetry.addData("dampedSine", dampedSine);
        panelsTelemetry.addData("lissajous", lissajous);
        panelsTelemetry.addData("ramp", ramp);
        panelsTelemetry.addData("square", squareWave);
        panelsTelemetry.addData("const", constVariable);

        // Multiple values in one line
        panelsTelemetry.addLine("extra1:" + t + " extra2:" + (t * t) + " extra3:" + sqrt(t));

        // Update the dashboard
        panelsTelemetry.update();
    }
}
