package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * FLYWHEEL kF CALIBRATION TOOL
 * * Instructions:
 * 1. Support the robot so the flywheel can spin safely.
 * 2. Use a charged battery (12.5V+).
 * 3. Run the OpMode and press START.
 * 4. The robot will spin at two different voltages and calculate the normalized kF.
 * 5. Record the results from the screen/Dashboard.
 */

@TeleOp(name="Flywheel kF Calibration", group="Calibration")
public class AutoCalibration extends LinearOpMode {

    private DcMotorEx flywheel;
    private DcMotorEx flywheel2;
    private VoltageSensor batteryVoltageSensor;

    // We use a 12V reference for all our normalized math
    private final double VOLTAGE_REFERENCE = 12.0;

    @Override
    public void runOpMode() {
        // Initialize Hardware
        flywheel = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Ensure motor is in the right mode for raw testing
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        telemetry.addLine("--- Flywheel Calibration Tool ---");
        telemetry.addLine("1. Ensure wheel is clear.");
        telemetry.addLine("2. Use a fresh battery.");
        telemetry.addLine("Press START to begin sequence.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // --- STEP 1: LOW SPEED TEST (Targeting 4.0 Volts) ---
        double lowTargetVolts = 4.0;
        double kF_low = performTest(lowTargetVolts, "LOW SPEED");

        // --- STEP 2: COOL DOWN ---
        flywheel.setPower(0);
        flywheel2.setPower(0);
        telemetry.addLine("Cooling down for 3 seconds...");
        telemetry.update();
        sleep(3000);

        // --- STEP 3: HIGH SPEED TEST (Targeting 9.0 Volts) ---
        double highTargetVolts = 9.0;
        double kF_high = performTest(highTargetVolts, "HIGH SPEED");

        // --- STEP 4: FINAL RESULTS ---
        flywheel.setPower(0);
        flywheel2.setPower(0);
        while (!isStopRequested()) {
            telemetry.addLine("--- CALIBRATION RESULTS ---");
            telemetry.addData("Calculated kF_LOW", "%.6f", kF_low);
            telemetry.addData("Calculated kF_HIGH", "%.6f", kF_high);
            telemetry.addLine("\nCopy these values into your production code.");
            telemetry.addLine("Press STOP to exit.");
            telemetry.update();
        }
    }

    /**
     * Spins the motor at a specific voltage, waits for stability, and calculates kF.
     */
    private double performTest(double targetVolts, String label) {
        telemetry.addData("Current Phase", label);

        // Stabilize for 4 seconds
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 12.0) {
            double vBat = batteryVoltageSensor.getVoltage();

            // Set power based on current battery to hit exactly targetVolts
            double powerToApply = targetVolts / vBat;
            flywheel.setPower(powerToApply);
            flywheel2.setPower(powerToApply);

            telemetry.addData("Phase", label);
            telemetry.addData("Target Volts", targetVolts);
            telemetry.addData("Actual V_Bat", vBat);
            telemetry.addData("Current TPS", flywheel2.getVelocity());
            telemetry.update();
        }

        // Capture data at steady state
        double finalTPS = flywheel2.getVelocity();

        // Formula: kF = (TargetVolts / ReferenceVolts) / MeasuredTPS
        // This normalizes the result to our 12V system.
        return (targetVolts / VOLTAGE_REFERENCE) / finalTPS;
    }
}
