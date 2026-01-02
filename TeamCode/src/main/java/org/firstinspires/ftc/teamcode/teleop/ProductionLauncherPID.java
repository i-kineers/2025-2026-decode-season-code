package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@TeleOp(name = "Production Flywheel System")
public class ProductionLauncherPID extends LinearOpMode {

        // --- 1. Motor & Hardware ---
        private DcMotorEx flywheel;
        private VoltageSensor batteryVoltage;

        // --- 2. Live Tuning Constants (FTC Dashboard) ---
        public double kP = 0.0065;
        public double kI = 0.0001;
        public double kD = 0.0004;

        // Normalized to 12V scale (found via Calibration)
        public double kF_LOW = 0.000725;  // For 1500 Wheel RPM
        public double kF_HIGH = 0.000500; // For 3300 Wheel RPM

        public double LOW_TARGET_TPS = 1050.0;
        public double HIGH_TARGET_TPS = 2310.0;

        // --- 3. Safety & Control Variables ---
        public double MAX_CURRENT = 8.5;
        public double DANGER_THRESHOLD = 0.93;
        public double RECOVERY_SLEW = 0.40; // Aggressive torque-based recovery

        private double integralSum = 0;
        private double lastError = 0;
        private double lastPower = 0;
        private ElapsedTime timer = new ElapsedTime();

        @Override
        public void runOpMode() {
            // Initialization
            flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
            batteryVoltage = hardwareMap.voltageSensor.iterator().next();
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Combine standard and dashboard telemetry


            waitForStart();

            while (opModeIsActive()) {
                // Select Target (D-Pad Control)
                double targetTPS = 1540;
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

                // 4. Current Watchdog (Safety)
                if (flywheel.getCurrent(CurrentUnit.AMPS) > MAX_CURRENT && currentTPS < 100) {
                    flywheel.setPower(0);
                    telemetry.addData("!!!", "STALL DETECTED");
                } else {
                    flywheel.setPower(Range.clip(finalPower, 0, 1));
                }

                // Dashboard Graphing
                telemetry.addData("Target", targetTPS);
                telemetry.addData("Actual", currentTPS);
                telemetry.addData("V_Battery", vBat);
                telemetry.update();
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
}