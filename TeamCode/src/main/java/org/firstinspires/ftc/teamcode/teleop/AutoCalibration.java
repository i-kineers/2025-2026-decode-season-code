package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Flywheel Auto Calibration (TPS)", group = "Calibration")
public class AutoCalibration extends LinearOpMode {

    private DcMotorEx flywheelMotor;
    // private DcMotorEx flywheelMotor2; 
    private VoltageSensor voltageSensor;
    private ElapsedTime timer = new ElapsedTime();

    // Configuration
    private static final double VOLTAGE_REF = 12.0;

    @Override
    public void runOpMode() {
        // 1. Hardware Init
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        // flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // flywheelMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Ready (TPS Mode)");
        telemetry.addData("Instructions", "Press A to start calibration");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                runAutoCalibration();
                // Wait until button release
                while (opModeIsActive() && gamepad1.a) { idle(); }
                
                // Reset telemetry after returning
                telemetry.addData("Status", "Ready (TPS Mode)");
                telemetry.addData("Instructions", "Press A to start calibration");
                telemetry.update();
            }
            idle();
        }
    }

    public void runAutoCalibration() {
        // --- STEP 1: Low Voltage Test (4V) ---
        double targetVoltageLow = 4.0;
        double currentBattery = getVoltage();
        double powerLow = targetVoltageLow / currentBattery;
        
        flywheelMotor.setPower(powerLow);
        // flywheelMotor2.setPower(powerLow);
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 12.0) {
            double currentTPS = Math.abs(flywheelMotor.getVelocity());
            telemetry.addData("Step 1", "Testing at %.1fV", targetVoltageLow);
            telemetry.addData("Target Power", "%.2f", powerLow);
            telemetry.addData("Current TPS", "%.0f", currentTPS);
            telemetry.update();
        }
        
        // Get TPS directly
        double tpsLow = Math.abs(flywheelMotor.getVelocity());

        // --- STEP 2: High Voltage Test (9V) ---
        double targetVoltageHigh = 9.0;
        currentBattery = getVoltage();
        double powerHigh = targetVoltageHigh / currentBattery;
        
        flywheelMotor.setPower(powerHigh);
        // flywheelMotor2.setPower(powerHigh);
        
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 12.0) {
            double currentTPS = Math.abs(flywheelMotor.getVelocity());
            telemetry.addData("Step 2", "Testing at %.1fV", targetVoltageHigh);
            telemetry.addData("Target Power", "%.2f", powerHigh);
            telemetry.addData("Current TPS", "%.0f", currentTPS);
            telemetry.update();
        }
        
        // Get TPS directly
        double tpsHigh = Math.abs(flywheelMotor.getVelocity());

        // --- STEP 3: Stop & Calculate ---
        flywheelMotor.setPower(0);
        // flywheelMotor2.setPower(0);

        // --- CALCULATIONS ---
        double normPowerLow = targetVoltageLow / VOLTAGE_REF;
        double normPowerHigh = targetVoltageHigh / VOLTAGE_REF;

        double kF = 0;
        double kS = 0;

        if (Math.abs(tpsHigh - tpsLow) > 10.0) {
            kF = (normPowerHigh - normPowerLow) / (tpsHigh - tpsLow);
            kS = normPowerHigh - (kF * tpsHigh);
        }

        // --- DISPLAY RESULTS ---
        // Loop to keep results on screen
        while (opModeIsActive()) {
            telemetry.addLine("=== CALIBRATION COMPLETE (TPS) ===");
            telemetry.addData("kF (Power/TPS)", "%.8f", kF);
            telemetry.addData("kS (Power)", "%.4f", kS);
            telemetry.addData("Low TPS", "%.0f", tpsLow);
            telemetry.addData("High TPS", "%.0f", tpsHigh);
            telemetry.addLine("---------------------------");
            telemetry.addLine("Press B to Exit");
            telemetry.update();
            
            if (gamepad1.b) break;
        }
    }

    private double getVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) result = Math.min(result, v);
        }
        return (result == Double.POSITIVE_INFINITY) ? 12.0 : result;
    }
}
