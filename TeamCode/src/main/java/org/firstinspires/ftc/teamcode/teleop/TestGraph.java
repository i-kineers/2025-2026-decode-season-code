package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.*;

@TeleOp(name = "Test Graph")
public class TestGraph extends OpMode {
    private ElapsedTime timer = new ElapsedTime();

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

        // Send data to FTC telemetry
        telemetry.addData("sin", sinVariable);
        telemetry.addData("cos", cosVariable);
        telemetry.addData("dampedSine", dampedSine);
        telemetry.addData("lissajous", lissajous);
        telemetry.addData("ramp", ramp);
        telemetry.addData("square", squareWave);
        telemetry.addData("const", constVariable);

        // Multiple values in one line
        telemetry.addData("extra", "t=" + t + " t^2=" + (t*t) + " sqrt(t)=" + sqrt(t));

        telemetry.update();
    }
}
