package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OuttakePID {

    // ===== MOTOR CONSTANTS =====
    private static final double TICKS_PER_REV = 28.0;  // GoBilda 6000 RPM motor
    private static final double MAX_RPM = 6000.0;

    // ===== PID CONSTANTS =====
    public static double kP = 0.01;
    public static double kI = 0.0002;
    public static double kD = 0.0004;
    public static double kF = 0.78 / MAX_RPM;

    public static double SMOOTHING_ALPHA = 0.2;
    public static double RPM_TOLERANCE = 15.0;

    // ===== RUNTIME STATE =====
    private double targetRPM = 0.0;
    private double currentRPM = 0.0;

    private double p_component = 0.0;
    private double i_component = 0.0;
    private double d_component = 0.0;
    private double raw_pid_value = 0.0;
    private double previous_error = 0.0;
    private double previous_time = 0.0;
    private double lastCalculatedPower = 0.0;
    private double lastFilteredRPM = 0.0;

    // ===== HARDWARE =====
    private final DcMotorEx launcher;
    private final CRServo leftLoader;
    private final CRServo rightLoader;

    // ===== TIMERS =====
    private final ElapsedTime runtime;
    private final ElapsedTime loopTimer;

    // FSM
    public enum ShooterState {
        STOPPED,      // Not running
        READY,        // Spinning up to the target RPM
        SHOOTING      // At target RPM and trigger is active (loaders are on)
    }
    private ShooterState currentState = ShooterState.STOPPED;
    private static boolean launcherReady = false;

    // ===== CONSTRUCTOR =====
    public OuttakePID(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftLoader = hardwareMap.get(CRServo.class, "leftLoader");
        rightLoader = hardwareMap.get(CRServo.class, "rightLoader");

        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        runtime = new ElapsedTime();
        loopTimer = new ElapsedTime();
    }

    // Shooter Logic
    public void startShooterLogic() {
        switch (currentState) {
            case STOPPED:
                stop();
                break;
            case READY:
                setTargetRPM(3500);
                break;
            case SHOOTING:
                // Check if we're near target RPM
                if (Math.abs(getCurrentRPM() - targetRPM) <= 50) {
                    pullTrigger();
                    launcherReady = true;
                } else {
                    releaseTrigger();
                    launcherReady = false;
                }
                break;
        }
    }

    // State Switcher
    public void nextState() {
        switch (currentState) {
            case STOPPED:
                currentState = ShooterState.READY;
                break;
            case READY:
                currentState = ShooterState.SHOOTING;
                break;
            case SHOOTING:
                currentState = ShooterState.STOPPED;
                break;
        }
    }


    // ===== MAIN PID METHOD =====
    private double runLauncherPID() {
        double now = runtime.milliseconds();
        double dt = now - previous_time;

        if (previous_time == 0 || dt <= 0) {
            previous_time = now;
            return lastCalculatedPower;
        }

        // --- Compute current RPM from motor velocity ---
        double rawRPM = (launcher.getVelocity() / TICKS_PER_REV) * 60.0;
        currentRPM = (SMOOTHING_ALPHA * rawRPM) + ((1.0 - SMOOTHING_ALPHA) * lastFilteredRPM);
        lastFilteredRPM = currentRPM;

        // --- PID Error ---
        double error = targetRPM - currentRPM;
        if (Math.abs(error) < RPM_TOLERANCE) error = 0;

        double dtSec = dt / 1000.0;

        // --- PID Components ---
        p_component = kP * error;

        i_component += kI * (error * dtSec);
        i_component = Math.max(-3000, Math.min(i_component, 3000));  // anti-windup

        d_component = kD * (error - previous_error) / dtSec;

        double f_component = kF * targetRPM;

        // --- Combine PID + Feedforward ---
        raw_pid_value = p_component + i_component + d_component + f_component;

        double power = Math.max(0.0, Math.min(raw_pid_value, 1.0));

        // --- Save for next loop ---
        lastCalculatedPower = power;
        previous_error = error;
        previous_time = now;
        loopTimer.reset();

        return power;
    }

    // ===== UPDATE LOOP =====
    public void update() {
        if (targetRPM <= 0) {
            launcher.setPower(0);
        } else {
            double power = runLauncherPID();
            launcher.setPower(power);
        }
    }

    // ===== RPM CONTROL =====
    public void setTargetRPM(double rpm) {
        this.targetRPM = Math.max(0, Math.min(MAX_RPM, rpm));
    }

    public double getTargetRPM() {
        return this.targetRPM;
    }

    public double getCurrentRPM() {
        return this.currentRPM;
    }

    public ShooterState getState() {
        return currentState;
    }

    public double getCalculatedPower() {
        return this.lastCalculatedPower;
    }

    public double getPID() {
        return this.raw_pid_value;
    }


    // --- Accessors for Panels + tuning methods ---
    public double getP() { return kP; }
    public double getI() { return kI; }
    public double getD() { return kD; }
    public double getF() { return kF; }

    public void increaseP() { kP += 0.0001; }
    public void decreaseP() { kP = Math.max(0, kP - 0.0001); }

    public void increaseI() { kI += 0.00005; }
    public void decreaseI() { kI = Math.max(0, kI - 0.00005); }

    public void increaseD() { kD += 0.00005; }
    public void decreaseD() { kD = Math.max(0, kD - 0.00005); }

    public void increaseF() { kF += (0.78 / MAX_RPM) * 0.05; } // +5%
    public void decreaseF() { kF = Math.max(0, kF - (0.78 / MAX_RPM) * 0.05); } // -5%

    // ===== SHOOTER CONTROL =====
    public void pullTrigger() {
        leftLoader.setPower(1.0);
        rightLoader.setPower(-1.0);
    }

    public void releaseTrigger() {
        leftLoader.setPower(0.0);
        rightLoader.setPower(0.0);
    }

    // ===== STOP =====
    public void stop() {
        releaseTrigger();
        launcher.setPower(0);
        targetRPM = 0;
        p_component = 0;
        i_component = 0;
        d_component = 0;
        previous_error = 0;
        previous_time = 0;
    }

    // ===== SLEEP =====
    public void sleep(long milli) {
        try {
            Thread.sleep(milli);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // Accessor methods
    public double getRawPID() { return raw_pid_value; }  // unclamped
    public double getMotorPower() { return lastCalculatedPower; }  // clamped 0–1

    // ===== LEGACY COMPATIBILITY METHODS =====
    // (so your TeleOp doesn't break)
    public double getTargetPosition() { return targetRPM; }
    public double getCurrentPosition() { return currentRPM; }
    public double getRPM() { return currentRPM; }
}
