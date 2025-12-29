package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Flywheel Tuner Tutorial", group = "Tuning")
public class FlywheelTunerTutorial extends OpMode {

    private PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    public DcMotorEx flywheelMotor;
    public DcMotorEx flywheelMotor2;


    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1,0.01, 0.001, 0.0001};

    int stepIndex = 1;

    private static final double TICKS_PER_REV = 28.0;  // GoBilda 6000 RPM motor
    public static double SMOOTHING_ALPHA = 0.2;
    private double lastFilteredRPM = 0.0;
    private double currentRPM = 0.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheelMotor.setVelocity(curTargetVelocity);
        flywheelMotor2.setVelocity(curTargetVelocity);

        double curVelocity = flywheelMotor.getVelocity();

        double error = curTargetVelocity - curVelocity;

        // ---Compute Target RPM---
        double targetRPM = (curTargetVelocity / TICKS_PER_REV) * 60.0;

        // --- Compute current RPM from motor velocity ---
        double rawRPM = (flywheelMotor.getVelocity() / TICKS_PER_REV) * 60.0;
        currentRPM = (SMOOTHING_ALPHA * rawRPM) + ((1.0 - SMOOTHING_ALPHA) * lastFilteredRPM);
        lastFilteredRPM = currentRPM;

        telemetry.addData("Target Veclocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", Math.abs(curVelocity));
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Current RPM", "%.2f", Math.abs(currentRPM));
        telemetry.addData("Target RPM", "%.2f", targetRPM);
        telemetry.addLine("------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

        panelsTelemetry.getTelemetry().addData("Target Velocity", curTargetVelocity);
        panelsTelemetry.getTelemetry().addData("Current Velocity", Math.abs(curVelocity));
        panelsTelemetry.getTelemetry().addData("Target RPM", targetRPM);
        panelsTelemetry.getTelemetry().addData("Current RPM", Math.abs(currentRPM));
        telemetry.addData("Target RPM", "%.2f", targetRPM);
        panelsTelemetry.getTelemetry().addData("Error", error);
        panelsTelemetry.getTelemetry().addData("Tuning P", P);
        panelsTelemetry.getTelemetry().addData("Tuning F", F);
        panelsTelemetry.getTelemetry().addData("Step Size", stepSizes[stepIndex]);
        panelsTelemetry.getTelemetry().update();
    }
}
