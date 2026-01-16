package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ILTFlywheelTunerSubsystem {
    private PanelsTelemetry panelsTelemetry;
    public DcMotorEx flywheelMotor;
    public DcMotorEx flywheelMotor2;

    // Target Velocity in TICKS PER SECOND
    public double targetVelocity = 1000;

    // PIDF for built-in controller (Steady State)
    double F = 0;
    double P = 0;

    // Tuning step size
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;

    // RPM calculation constants
    private static final double TICKS_PER_REV = 28.0;
    public static double SMOOTHING_ALPHA = 0.2;
    private double lastFilteredRPM = 0.0;
    private double currentRPM = 0.0;

    // Input tracking for single presses
    boolean previousB = false;
    boolean previousDpadLeft = false;
    boolean previousDpadRight = false;
    boolean previousDpadUp = false;
    boolean previousDpadDown = false;
    boolean previousRightBumper = false;
    boolean previousLeftBumper = false;

    // "Shotware" Logic Variables
    private boolean isRecovering = false;
    private double recoveryKP = 0.002; // Aggressive P for manual recovery
    private double recoveryKF = 0.0001; // Feedforward for manual recovery

    public ILTFlywheelTunerSubsystem(HardwareMap hardwareMap) {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void updateShooter(double targetTicksPerSec) {
        // This function implements the "Shotware" concept.
        // It uses the built-in PID for steady speed, but when a shot is detected (sudden RPM drop),
        // it switches to a more aggressive manual power controller to recover speed quickly.

        double currentVelocity = flywheelMotor.getVelocity();
        currentRPM = (currentVelocity / TICKS_PER_REV) * 60.0;
        double targetRPM = (targetTicksPerSec / TICKS_PER_REV) * 60.0;

        // --- 1. SHOT DETECTION ---
        // If we are in steady state and RPM drops below 92% of target, enter recovery mode.
        if (!isRecovering && currentRPM < (targetRPM * 0.92) && targetRPM > 0) {
            isRecovering = true;
            // Switch to manual power mode for aggressive recovery
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // --- 2. STATE LOGIC ---
        if (isRecovering) {
            // --- AGGRESSIVE RECOVERY ---
            double error = targetRPM - currentRPM;
            // A simple manual P + F controller for raw power
            double power = (error * recoveryKP) + (targetRPM * recoveryKF);
            power = Math.min(power, 1.0); // Clamp power to 1.0

            flywheelMotor.setPower(power);
            flywheelMotor2.setPower(power);

            // --- Check if we should exit recovery mode ---
            if (currentRPM >= (targetRPM * 0.98)) {
                isRecovering = false;
                // Switch back to the built-in PID controller mode
                flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        } else {
            // --- STEADY STATE ---
            // Use the built-in PID controller that is being tuned.
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            flywheelMotor.setVelocity(targetTicksPerSec);
            flywheelMotor2.setVelocity(targetTicksPerSec);
        }
    }

    public void handleGamepadInputs(Gamepad gamepad1) {
        // Adjust Target Velocity with Bumpers
        if (gamepad1.right_bumper && !previousRightBumper) {
            targetVelocity += 100;
        }
        previousRightBumper = gamepad1.right_bumper;

        if (gamepad1.left_bumper && !previousLeftBumper) {
            targetVelocity -= 100;
        }
        previousLeftBumper = gamepad1.left_bumper;

        // Cycle Step Size with B
        if (gamepad1.b && !previousB) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        previousB = gamepad1.b;

        // Tune F with D-Pad Left/Right
        if (gamepad1.dpad_left && !previousDpadLeft) { F -= stepSizes[stepIndex]; }
        previousDpadLeft = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !previousDpadRight) { F += stepSizes[stepIndex]; }
        previousDpadRight = gamepad1.dpad_right;

        // Tune P with D-Pad Up/Down
        if (gamepad1.dpad_down && !previousDpadDown) { P -= stepSizes[stepIndex]; }
        previousDpadDown = gamepad1.dpad_down;

        if (gamepad1.dpad_up && !previousDpadUp) { P += stepSizes[stepIndex]; }
        previousDpadUp = gamepad1.dpad_up;
    }

    public void updateTelemetry(Telemetry telemetry) {
        double currentVelocity = flywheelMotor.getVelocity();
        double error = targetVelocity - currentVelocity;
        double targetRPM = (targetVelocity / TICKS_PER_REV) * 60.0;

        telemetry.addData("Target Velocity (Ticks/s)", targetVelocity);
        telemetry.addData("Current Velocity (Ticks/s)", "%.2f", currentVelocity);
        telemetry.addData("Error (Ticks/s)", "%.2f", error);
        telemetry.addData("Target RPM", "%.2f", targetRPM);
        telemetry.addData("Current RPM", "%.2f", currentRPM);
        telemetry.addData("State", isRecovering ? "RECOVERING" : "STEADY");
        telemetry.addLine("------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.addData("Adjust Target", "Bumpers (+/- 100)");
        telemetry.update();

        panelsTelemetry.getTelemetry().addData("Target Velocity", targetVelocity);
        panelsTelemetry.getTelemetry().addData("Current Velocity", currentVelocity);
        panelsTelemetry.getTelemetry().addData("Target RPM", targetRPM);
        panelsTelemetry.getTelemetry().addData("Current RPM", currentRPM);
        panelsTelemetry.getTelemetry().addData("Error", error);
        panelsTelemetry.getTelemetry().addData("Tuning P", P);
        panelsTelemetry.getTelemetry().addData("Tuning F", F);
        panelsTelemetry.getTelemetry().update();
    }
}