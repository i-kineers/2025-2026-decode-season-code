package org.firstinspires.ftc.teamcode.teleop;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import java.sql.Date;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Production Flywheel System with shot awareness")
public class ProdFlywheelPIDWithShotAwareness extends LinearOpMode {


    // --- 1. Motor & Hardware ---
    private DcMotorEx flywheel;
    private DcMotorEx intake; // Added intake variable
    private VoltageSensor batteryVoltage;


    // --- 2. Live Tuning Constants (FTC Dashboard) ---
    public double kP = 0.001;
    public double kI = 0.0001;
    public double kD = 0.0004;


    // Normalized to 12V scale (found via Calibration)
    public double kF_LOW = 0.000725;  // For 1500 Wheel RPM
    public double kF_HIGH = 0.000500; // For 3300 Wheel RPM


    public double LOW_TARGET_TPS = 1213.0;
    public double HIGH_TARGET_TPS = 1540.0;


    // --- 3. Safety & Control Variables ---
    public double MAX_CURRENT = 8.5;
    public double DANGER_THRESHOLD = 0.93;
    public double RECOVERY_SLEW = 1.0; // Aggressive torque-based recovery


    private double integralSum = 0;
    private double lastError = 0;

    private double lastPower = 0;
    private ElapsedTime timer = new ElapsedTime();
    private enum ShotState { IDLE, FIRING, RECOVERING }
    ShotState currentShotState = ShotState.IDLE;
    private ElapsedTime recoveryTimer = new ElapsedTime();
    private double lastRecoveryTime = 0;
    private boolean isRecovering = false;

    int shotsFired = 0;
    public static int TARGET_SHOT_COUNT = 3; // Number of rings/balls to fire
    ElapsedTime shotTimer = new ElapsedTime();
    private List<Double> recoveryLog = new ArrayList<>();
    private String logFileName = "FlywheelPerformance.txt";



    @Override
    public void runOpMode() {
        // Initialization
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        // Initialize intake with error handling
        try {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
        } catch (Exception e) {
            telemetry.addData("Warning", "Intake not found in config");
        }


        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Combine standard and dashboard telemetry




        waitForStart();


        while (opModeIsActive()) {
            // Select Target (D-Pad Control)
            double targetTPS = 1213;
            if (gamepad1.dpad_up) targetTPS = HIGH_TARGET_TPS;
            else if (gamepad1.dpad_down) targetTPS = LOW_TARGET_TPS;


            double currentTPS = flywheel.getVelocity();
            double vBat = batteryVoltage.getVoltage();


            // 1. Calculate Voltage-Agnostic PIDF
            double rawPower = calculatePIDF(targetTPS, currentTPS);


            // 2. Apply Battery Compensation (Reference to 12V)
            double compensatedPower = rawPower * (12.0 / vBat);


            // 3. Apply Slew Rate
            double finalPower = applySlew(compensatedPower);
            double speedRatio = currentTPS / targetTPS;

// 1. Detect the start of a recovery (The Shot)
            if (!isRecovering && speedRatio < DANGER_THRESHOLD) {
                isRecovering = true;
                recoveryTimer.reset();
            }

// 2. Detect the end of recovery (Back to Speed)
            if (isRecovering && speedRatio >= 0.98) {
                isRecovering = false;
                lastRecoveryTime = recoveryTimer.milliseconds();
                recoveryLog.add(lastRecoveryTime); // Save this shot's time

            }

// 3. Add to Telemetry/Dashboard
            telemetry.addData("Recovery Time (ms)", lastRecoveryTime);


            // 4. Current Watchdog (Safety)
            if (flywheel.getCurrent(CurrentUnit.AMPS) > MAX_CURRENT && currentTPS < 100) {
                flywheel.setPower(0);
                telemetry.addData("!!!", "STALL DETECTED");
            } else {
                flywheel.setPower(Range.clip(finalPower, 0, 1));
            }


            // --- Auto Shot Recovery Logic ---
            boolean intakeAllowed = currentTPS >= (targetTPS * 0.95);


            if (intake != null ) {
                switch (currentShotState) {
                    case IDLE:
                        intake.setPower(0);
                        // Wait for Driver to start the sequence
                        if (gamepad1.right_bumper) {
                            shotsFired = 0;
                            currentShotState = ShotState.FIRING;
                        }
                        break;


                    case FIRING:
                        // The wheel is ready (from our previous intakeAllowed logic)
                        if (intakeAllowed) {
                            intake.setPower(1.0);
                            // Detection: If speed drops significantly, a shot just happened
                            if (currentTPS < (targetTPS * DANGER_THRESHOLD)) {
                                shotsFired++;
                                currentShotState = ShotState.RECOVERING;
                                shotTimer.reset();
                            }
                        }
                        break;


                    case RECOVERING:
                        intake.setPower(0); // Stop intake to let wheel recover
                        // Wait for recovery AND a small delay so pieces don't jam
                        if (intakeAllowed && shotTimer.milliseconds() > 100) {
                            if (shotsFired >= TARGET_SHOT_COUNT) {
                                currentShotState = ShotState.IDLE;
                            } else {
                                currentShotState = ShotState.FIRING;
                            }
                        }
                        break;
                }
            }


            // Dashboard Graphing
            telemetry.addData("Target", targetTPS);
            telemetry.addData("Actual", currentTPS);
            telemetry.addData("V_Battery", vBat);
            telemetry.addData("State", currentShotState);
            telemetry.update();
            saveLog();
        }
    }




    private double calculatePIDF(double target, double current) {
        if (target <= 0) return 0;


        // Dynamic kF Interpolation
        double slope = (kF_HIGH - kF_LOW) / (HIGH_TARGET_TPS - LOW_TARGET_TPS);
        double dynamicKF = kF_LOW + (target - LOW_TARGET_TPS) * slope;
        double feedforward = target * dynamicKF;


        double error = target - current;


        // Bang-Bang for extreme recovery
        if (current < target * DANGER_THRESHOLD) return 1.0;


        // PID Math
        double dt = timer.seconds();
        if (dt < 0.001) dt = 0.001;


        if (Math.abs(error) < 150) integralSum += error * dt;
        else integralSum = 0;


        double derivative = (error - lastError) / dt;
        lastError = error;
        timer.reset();


        return feedforward + (kP * error) + (kI * integralSum) + (kD * derivative);
    }


    private double applySlew(double targetPower) {
        double delta = targetPower - lastPower;
        if (Math.abs(delta) > RECOVERY_SLEW) {
            targetPower = lastPower + (Math.signum(delta) * RECOVERY_SLEW);
        }
        lastPower = targetPower;
        return targetPower;
    }
    private void saveLog() {
        if (recoveryLog.isEmpty()) return;

        double sum = 0;
        for (double time : recoveryLog) sum += time;
        double average = sum / recoveryLog.size();

        // Use standard Java File I/O to save to the Robot Controller's storage
        try {
            File file = new File(AppUtil.getInstance().getSettingsFile("Log"), logFileName);
            FileWriter writer = new FileWriter(file, true); // 'true' appends to the file
            writer.write(String.format("Match Date: %s | Shots: %d | Avg Recovery: %.2f ms\n ",
                    new Date(1/10/25).toString(), recoveryLog.size(), average));
            writer.close();
        } catch (IOException e) {
            telemetry.addData("Log Error", e.getMessage());
            telemetry.update();
        }
    }

}
