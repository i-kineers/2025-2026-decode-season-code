package org.firstinspires.ftc.teamcode.teleop;

import android.os.Environment;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@TeleOp(name = "Flywheel Tuner Tutorial", group = "Tuning")
public class FlywheelTunerTutorial extends OpMode {

    private PanelsTelemetry panelsTelemetry;
    public DcMotorEx flywheelMotor;
    public DcMotorEx flywheelMotor2;

    // Target Velocity in TICKS PER SECOND
    public double targetVelocity = 1000;

    // PIDF for built-in controller (Steady State)
    double F = 0;
    double P = 0;
    double StaticPower = 4;

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
    boolean previousA = false;
    boolean previousX = false;

    // "Shotware" Logic Variables
    private boolean isShooting = false; // Toggle for motor on/off
    private boolean isRecoveryEnabled = false; // Toggle for recovery logic (Default OFF for tuning)
    private boolean isRecovering = false;
    private double recoveryKP = 0.002; // Aggressive P for manual recovery
    private double recoveryKF = 0.0001; // Feedforward for manual recovery

    // Data Logging
    private long recoveryStartTime = 0;
    private double lastRecoveryDuration = 0.0;
    private static final String LOG_FILE_PATH = Environment.getExternalStorageDirectory().getPath() + "/FIRST/flywheel_recovery_data.csv";

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "launcher2");

       flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Log File
        initLogFile();

        telemetry.addLine("Init Complete");
        telemetry.addLine("Press A to Toggle Motor");
        telemetry.addLine("Press X to Toggle Recovery Mode");
        telemetry.addData("Log File", LOG_FILE_PATH);
    }

    @Override
    public void loop() {
        // --- Input Handling ---
        handleGamepadInputs();

        // --- Main Logic ---
        updateShooter(targetVelocity);

        // --- Telemetry ---
        updateTelemetry();
    }

    public void updateShooter(double targetTicksPerSec) {
        double currentVelocity = flywheelMotor2.getVelocity();
        // Absolute RPM for display and logic
        currentRPM = Math.abs((currentVelocity / TICKS_PER_REV) * 60.0);
        double targetRPM = (targetTicksPerSec / TICKS_PER_REV) * 60.0;

        if (isShooting) {
            // --- 1. SHOT DETECTION ---
            // Only enter recovery if enabled and RPM drops significantly
            if (isRecoveryEnabled && currentRPM < (targetRPM * 0.92) && targetRPM > 0) {
                isRecovering = true;
                recoveryStartTime = System.currentTimeMillis(); // Start Timer
                flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // --- 2. STATE LOGIC ---
            if (isRecovering) {
                // --- AGGRESSIVE RECOVERY ---
                double error = targetRPM - currentRPM;
                double power = (error * recoveryKP) + (targetRPM * recoveryKF);
                power = Math.min(power, 1.0);

                flywheelMotor.setPower(power);
               flywheelMotor2.setPower(power);

                if (currentRPM >= (targetRPM * 0.98)) {
                    isRecovering = false;

                    // Stop Timer and Log Data
                    long recoveryEndTime = System.currentTimeMillis();
                    lastRecoveryDuration = (recoveryEndTime - recoveryStartTime) / 1000.0; // Convert to seconds
                    logRecoveryData(lastRecoveryDuration, targetRPM, recoveryKP, recoveryKF);

                    flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            } else {
                // --- STEADY STATE ---
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
                flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

                flywheelMotor.setVelocity(targetTicksPerSec);
                flywheelMotor2.setVelocity(targetTicksPerSec);
            }
        } else {
            //flywheelMotor.setPower(0);
            flywheelMotor2.setPower(0);
            isRecovering = false;
        }
    }

    private void handleGamepadInputs() {
        // Toggle Motor with A
        if (gamepad1.a && !previousA) {
            isShooting = !isShooting;
        }
        previousA = gamepad1.a;

        // Toggle Recovery Mode with X
        if (gamepad1.x && !previousX) {
            isRecoveryEnabled = !isRecoveryEnabled;
        }
        previousX = gamepad1.x;

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

    private void updateTelemetry() {
        double currentVelocity = flywheelMotor2.getVelocity();
        double error = targetVelocity - currentVelocity;
        double targetRPM = (targetVelocity / TICKS_PER_REV) * 60.0;

        telemetry.addData("Target Velocity (Ticks/s)", targetVelocity);
        telemetry.addData("Current Velocity (Ticks/s)", "%.2f", currentVelocity);
        telemetry.addData("Error (Ticks/s)", "%.2f", error);
        telemetry.addData("Target RPM", "%.2f", targetRPM);
        telemetry.addData("Current RPM", "%.2f", currentRPM);
        telemetry.addData("Motor State", isShooting ? "ON" : "OFF");
        telemetry.addData("Recovery Mode", isRecoveryEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("State", isRecovering ? "RECOVERING" : "STEADY");
        telemetry.addData("Last Recovery Time", "%.3f sec", lastRecoveryDuration);
        telemetry.addLine("------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.update();

        panelsTelemetry.getTelemetry().addData("Target Velocity", targetVelocity);
        panelsTelemetry.getTelemetry().addData("Current Velocity", currentVelocity);
        panelsTelemetry.getTelemetry().addData("Target RPM", targetRPM);
        panelsTelemetry.getTelemetry().addData("Current RPM", currentRPM);
        panelsTelemetry.getTelemetry().addData("Error", error);
        panelsTelemetry.getTelemetry().addData("Tuning P", P);
        panelsTelemetry.getTelemetry().addData("Tuning F", F);
        panelsTelemetry.getTelemetry().addData("Recovery Time", lastRecoveryDuration);
        panelsTelemetry.getTelemetry().update();
    }

    // --- Data Logging Methods ---

    private void initLogFile() {
        try {
            File file = new File(LOG_FILE_PATH);
            // Create directory if it doesn't exist
            file.getParentFile().mkdirs();

            // If file doesn't exist, create it and add header
            if (!file.exists()) {
                FileWriter writer = new FileWriter(file);
                writer.append("TargetRPM,RecoveryTime(s),RecoveryKP,RecoveryKF\n");
                writer.close();
            }
        } catch (IOException e) {
            telemetry.addData("Log Init Error", e.getMessage());
        }
    }

    private void logRecoveryData(double time, double target, double p, double f) {
        try {
            File file = new File(LOG_FILE_PATH);
            FileWriter writer = new FileWriter(file, true); // Append mode
            writer.append(String.format(Locale.US, "%.1f,%.3f,%.5f,%.5f\n", target, time, p, f));
            writer.close();
        } catch (IOException e) {
            telemetry.addData("Log Write Error", e.getMessage());
        }
    }
}
