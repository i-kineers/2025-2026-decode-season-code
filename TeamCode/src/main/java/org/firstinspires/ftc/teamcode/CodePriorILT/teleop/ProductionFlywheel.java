package org.firstinspires.ftc.teamcode.CodePriorILT.teleop;

import android.os.Environment;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;
import java.util.TreeMap;
import java.util.Map;

/**
 * Production-Ready Flywheel Controller
 * Features:
 * 1. Voltage-Agnostic Feedforward (Auto-Calibration)
 * 2. Slew Rate Limiter (Protects gears/belts)
 * 3. Distance-based RPM Interpolation
 * 4. "Shotware" Recovery Logic (Fast recovery after shots)
 * 5. Safety Watchdog (Stall detection)
 * 6. Data Logging
 */
@TeleOp(name = "Production Flywheel", group = "Production")
public class ProductionFlywheel extends OpMode {

    // ==================================================================
    // CONFIGURATION & CONSTANTS
    // ==================================================================
    
    // Hardware
    private DcMotorEx flywheelMotor, flywheelMotor2;
    private VoltageSensor voltageSensor;
    private PanelsTelemetry panelsTelemetry;

    // Physics Constants (1:1 Gear Ratio, GoBilda 6000 RPM)
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_PHYSICAL_RPM = 6000.0;
    private static final double NOMINAL_VOLTAGE = 12.0; // Voltage the PID was tuned at

    // Tuning Constants (Steady State PID)
    public static double kP = 0.002; 
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 1.0 / MAX_PHYSICAL_RPM; // Base Feedforward

    // Recovery Constants (Manual Power Mode)
    public static double kS = 0.05; // Static Friction (Min power to move)
    public static double RECOVERY_kP = 0.003; // Aggressive P for recovery
    public static double RECOVERY_kF = 0.00018; // Aggressive F for recovery

    // Slew Rate (RPM per Second)
    public static double MAX_ACCEL_RPM_PER_SEC = 3000.0; 

    // Watchdog
    private static final double STALL_TIMEOUT_MS = 2000; // 2 seconds
    private static final double STALL_RPM_THRESHOLD = 100;

    // ==================================================================
    // VARIABLES
    // ==================================================================

    // State
    private double currentBatteryVoltage = 12.0;
    private double targetRPM = 0.0;
    private double currentRPM = 0.0;
    private double slewLimitedTargetRPM = 0.0;
    
    // Interpolation (Distance -> RPM)
    private TreeMap<Double, Double> rpmTable = new TreeMap<>();
    private double simulatedDistance = 10.0; // Feet

    // Logic Flags
    private boolean isMotorOn = false;
    private boolean isRecoveryActive = false;
    private boolean isWatchdogTripped = false;

    // Timers
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime watchdogTimer = new ElapsedTime();
    private double lastLoopTime = 0;

    // Logging
    private static final String LOG_PATH = Environment.getExternalStorageDirectory().getPath() + "/FIRST/production_flywheel_log.csv";

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE;
        
        // 1. Hardware Map
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // 2. Motor Config
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 3. Initialize Interpolation Table (Distance in ft -> RPM)
        // Add your specific measurements here
        rpmTable.put(5.0, 2500.0);
        rpmTable.put(10.0, 3200.0);
        rpmTable.put(15.0, 3800.0);
        rpmTable.put(20.0, 4500.0);

        // 4. Init Log
        initLogFile();

        telemetry.addData("Status", "Production Ready");
        telemetry.addData("Voltage", getVoltage());
    }

    @Override
    public void loop() {
        if (isWatchdogTripped) {
            handleWatchdogTrip();
            return;
        }

        // 1. Read Sensors
        double now = loopTimer.seconds();
        double dt = now - lastLoopTime;
        lastLoopTime = now;
        
        currentBatteryVoltage = getVoltage();
        double velocityTicks = flywheelMotor.getVelocity();
        currentRPM = Math.abs((velocityTicks / TICKS_PER_REV) * 60.0);

        // 2. Handle Input
        handleInput(dt);

        // 3. Calculate Target (Interpolation + Slew Rate)
        double desiredRPM = isMotorOn ? getInterpolatedRPM(simulatedDistance) : 0;
        
        // Apply Slew Rate Limiter
        double maxChange = MAX_ACCEL_RPM_PER_SEC * dt;
        if (slewLimitedTargetRPM < desiredRPM) {
            slewLimitedTargetRPM = Math.min(slewLimitedTargetRPM + maxChange, desiredRPM);
        } else if (slewLimitedTargetRPM > desiredRPM) {
            slewLimitedTargetRPM = Math.max(slewLimitedTargetRPM - maxChange, desiredRPM);
        }

        // 4. Control Loop (The "Brain")
        runControlLoop(slewLimitedTargetRPM, currentRPM);

        // 5. Safety Watchdog
        updateWatchdog();

        // 6. Telemetry & Logging
        updateTelemetry();
    }

    // ==================================================================
    // CONTROL LOGIC
    // ==================================================================

    private void runControlLoop(double target, double current) {
        // Thresholds
        double recoveryThreshold = target * 0.92; // Drop below 92% triggers recovery
        double recoveryExit = target * 0.98;      // Recover to 98% exits recovery

        // State Machine
        if (target == 0) {
            // IDLE
            isRecoveryActive = false;
            flywheelMotor.setPower(0);
            flywheelMotor2.setPower(0);
            return;
        }

        if (!isRecoveryActive && current < recoveryThreshold) {
            // Trigger Recovery
            isRecoveryActive = true;
            // Switch to manual power for direct voltage control
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (isRecoveryActive && current >= recoveryExit) {
            // Exit Recovery
            isRecoveryActive = false;
            // Switch back to PID
            flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (isRecoveryActive) {
            // --- RECOVERY MODE (Voltage Compensated) ---
            double error = target - current;
            
            // Feedforward + Proportional + Static Friction
            double rawPower = (target * RECOVERY_kF) + (error * RECOVERY_kP) + kS;
            
            // Voltage Compensation: If battery is low (11V), we need MORE power than at 12V.
            // Formula: Power * (Nominal / Current)
            double compensatedPower = rawPower * (NOMINAL_VOLTAGE / currentBatteryVoltage);
            
            compensatedPower = Range.clip(compensatedPower, 0, 1.0);

            flywheelMotor.setPower(compensatedPower);
            flywheelMotor2.setPower(compensatedPower);
            
            // Log this event
            logData(target, current, compensatedPower, currentBatteryVoltage, "RECOVERY");

        } else {
            // --- STEADY STATE (Built-in PID) ---
            // Note: Built-in PID handles voltage internally to an extent, 
            // but we can adjust F if needed.
            
            PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
            flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            
            double targetTicksPerSec = (target / 60.0) * TICKS_PER_REV;
            flywheelMotor.setVelocity(targetTicksPerSec);
            flywheelMotor2.setVelocity(targetTicksPerSec);
        }
    }

    // ==================================================================
    // HELPER METHODS
    // ==================================================================

    private double getInterpolatedRPM(double distance) {
        // Linear Interpolation between two closest points in the TreeMap
        Map.Entry<Double, Double> floor = rpmTable.floorEntry(distance);
        Map.Entry<Double, Double> ceiling = rpmTable.ceilingEntry(distance);

        if (floor == null) return ceiling.getValue(); // Below min range
        if (ceiling == null) return floor.getValue(); // Above max range
        if (floor.equals(ceiling)) return floor.getValue(); // Exact match

        // Linear Interpolation Formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        double ratio = (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey());
        return floor.getValue() + ratio * (ceiling.getValue() - floor.getValue());
    }

    private void updateWatchdog() {
        // If we are trying to spin (Target > 500) but moving very slowly (< 100)
        if (slewLimitedTargetRPM > 500 && currentRPM < STALL_RPM_THRESHOLD) {
            if (watchdogTimer.seconds() > (STALL_TIMEOUT_MS / 1000.0)) {
                isWatchdogTripped = true;
                flywheelMotor.setPower(0);
                flywheelMotor2.setPower(0);
            }
        } else {
            watchdogTimer.reset();
        }
    }

    private void handleWatchdogTrip() {
        telemetry.clearAll();
        telemetry.addData("CRITICAL ERROR", "WATCHDOG TRIPPED");
        telemetry.addData("Reason", "Motor Stall Detected");
        telemetry.addData("Action", "Motors Disabled. Restart OpMode.");
        telemetry.update();
    }

    private double getVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) result = Math.min(result, v);
        }
        return (result == Double.POSITIVE_INFINITY) ? 12.0 : result;
    }

    private void handleInput(double dt) {
        // Toggle Motor
        if (gamepad1.a && !previousA) isMotorOn = !isMotorOn;
        previousA = gamepad1.a;

        // Simulate Distance (Triggers)
        if (gamepad1.right_trigger > 0.1) simulatedDistance += 10.0 * dt; // Increase distance
        if (gamepad1.left_trigger > 0.1) simulatedDistance -= 10.0 * dt; // Decrease distance
        simulatedDistance = Range.clip(simulatedDistance, 5.0, 20.0);
        
        // Reset Watchdog
        if (gamepad1.y) {
            isWatchdogTripped = false;
            watchdogTimer.reset();
        }
    }
    boolean previousA = false;

    // ==================================================================
    // LOGGING & TELEMETRY
    // ==================================================================

    private void updateTelemetry() {
        telemetry.addData("State", isRecoveryActive ? "RECOVERING" : "STEADY");
        telemetry.addData("Target RPM (Slew)", "%.1f", slewLimitedTargetRPM);
        telemetry.addData("Current RPM", "%.1f", currentRPM);
        telemetry.addData("Distance", "%.1f ft", simulatedDistance);
        telemetry.addData("Interpolated Target", "%.1f RPM", getInterpolatedRPM(simulatedDistance));
        telemetry.addData("Battery", "%.2f V", currentBatteryVoltage);
        telemetry.addData("Motor", isMotorOn ? "ON" : "OFF");
        telemetry.update();
        
        panelsTelemetry.getTelemetry().addData("Target", slewLimitedTargetRPM);
        panelsTelemetry.getTelemetry().addData("Actual", currentRPM);
        panelsTelemetry.getTelemetry().update();
    }

    private void initLogFile() {
        try {
            File file = new File(LOG_PATH);
            file.getParentFile().mkdirs();
            if (!file.exists()) {
                FileWriter writer = new FileWriter(file);
                writer.append("Timestamp,TargetRPM,CurrentRPM,AppliedPower,Voltage,State\n");
                writer.close();
            }
        } catch (IOException ignored) {}
    }

    private void logData(double target, double current, double power, double volts, String state) {
        try {
            FileWriter writer = new FileWriter(LOG_PATH, true);
            writer.append(String.format(Locale.US, "%.3f,%.1f,%.1f,%.3f,%.2f,%s\n", 
                    loopTimer.seconds(), target, current, power, volts, state));
            writer.close();
        } catch (IOException ignored) {}
    }
}
