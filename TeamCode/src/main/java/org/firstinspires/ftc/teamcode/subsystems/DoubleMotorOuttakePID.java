package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DoubleMotorOuttakePID {

    // ===== MOTOR CONSTANTS =====
    private static final double TICKS_PER_REV = 28.0;  // GoBilda 6000 RPM motor
    private static final double MAX_RPM = 6000.0;

    // ===== PID CONSTANTS =====
    public static double kP = 0.005; // Good value is 0.005
    public static double kI = 0.0002;
    public static double kD = 0.0005; // Good value is 0.0005
    public static double kF = 1 / MAX_RPM; // Adjust this tod fine tune the value which will hold the target RP

    public static double SMOOTHING_ALPHA = 0.2;
    public static double RPM_TOLERANCE = 15.0;

    // ===== RUNTIME STATE =====
    private double targetRPM = 0;
    private double storeTargetRPM = 3000;
    private double currentRPM = 0.0;

    private double p_component = 0.0;
    private double i_component = 0.0;
    private double d_component = 0.0;
    private double raw_pid_value = 0.0;
    private double previous_error = 0.0;
    private double previous_time = 0.0;
    private double lastCalculatedPower = 0.0;
    private double lastFilteredRPM = 0.0;
    private boolean rapidShoot = false;
    private boolean servoToggle = false;


    // ===== HARDWARE =====
    private final DcMotorEx launcher;
    private final DcMotorEx launcher2;
    private final CRServo leftLoader;
    private final CRServo rightLoader;

    // ===== TIMERS =====
    private final ElapsedTime runtime;
    private final ElapsedTime loopTimer;

    // FSM
    public enum ShooterState {
        STOPPED,      // Not running
        SHOOTING      // At target RPM and trigger is active (loaders are on)
    }
    private ShooterState currentState = ShooterState.STOPPED;

    // ===== CONSTRUCTOR =====
    public DoubleMotorOuttakePID(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        leftLoader = hardwareMap.get(CRServo.class, "leftLoader");
        rightLoader = hardwareMap.get(CRServo.class, "rightLoader");

        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        launcher2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        runtime = new ElapsedTime();
        loopTimer = new ElapsedTime();
    }

    // Shooter Logic
    public void startShooterLogic() {
        switch (currentState) {
            case STOPPED:
                stop();
                break;
            case SHOOTING:
                setTargetRPM(storeTargetRPM);
                if (servoToggle){
                    if (rapidShoot) {
                        runLoader();
                    } else if (!rapidShoot) {
                        // Check if we're near target RPM
                        if (Math.abs(getCurrentRPM() - targetRPM) <= 50) {
                            runLoader();
                        } else {
                            stopLoader();
                        }
                    }
                } else if (!servoToggle) {
                    stopLoader();
                }
                break;
        }
    }

    // State Switcher
    public void nextState() {
        switch (currentState) {
            case STOPPED:
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
        double rawRPM = (launcher2.getVelocity() / TICKS_PER_REV) * 60.0;
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

        return power;
    }

    // ===== UPDATE LOOP =====
    public void update() {
        if (targetRPM <= 0) {
            launcher.setPower(0);
            launcher2.setPower(0);
        } else {
            double power = runLauncherPID();
            launcher.setPower(power);
            launcher2.setPower(power);
        }
    }

    public void autoRapidShoot(double rpm, long time, double delay) {
        loopTimer.reset();
        setTargetRPM(rpm);

        // Wait for the launcher to reach the target speed before shooting.
        ElapsedTime timer = new ElapsedTime();
        // Give it up to 3 seconds to spin up.
        while (timer.milliseconds() < time) {
            if (loopTimer.milliseconds() > delay) {
                runLoader();
            }
            update(); // This needs to be called to update the motor power from the PID controller.
            sleep(10); // Small pause to prevent busy-waiting and allow other things to run.
        }
        stop();
        update(); // Apply the change to stop the motors.
    }

    public void autoShootDelay(double rpm, long time, double delay) {
        loopTimer.reset();
        setTargetRPM(rpm);

        ElapsedTime timer = new ElapsedTime();
        boolean shotAllowed = true;

        while (timer.milliseconds() < time) {

            if (loopTimer.milliseconds() > delay) {

                // RPM close enough to target
                boolean atSpeed = Math.abs(rpm - currentRPM) < 50;

                if (atSpeed && shotAllowed) {
                    runLoader();          // Fire ONE shot
                    shotAllowed = false;  // Lock until RPM drops
                } else {
                    stopLoader();
                }

                // Wait for RPM to dip before allowing another shot
                if (!atSpeed) {
                    shotAllowed = true;
                }
            }

            update();
            sleep(10);
        }

        stopLoader();
        stop();
        update();
    }


    public void toggleRapidShoot() {
        if (!rapidShoot) {
            rapidShoot = true;
        } else if (rapidShoot) {
            rapidShoot = false;
        }
    }

    public void toggleServos(int toggle) {
        if (toggle == 1) {
            servoToggle = true;
        } else if (toggle == 0) {
            servoToggle = false;
        }
    }

    // ===== RPM CONTROL =====
    public void setTargetRPM(double rpm) { this.targetRPM = Math.max(0, Math.min(MAX_RPM, rpm)); }

    public void increaseTargetRPM() { storeTargetRPM += 100; }

    public void decreaseTargetRPM() { storeTargetRPM -= 100; }

    public double getTargetRPM() {
        return this.targetRPM;
    }

    public double getCurrentRPM() {
        return Math.abs(this.currentRPM);
    }

    // The new, SAFE method in DoubleMotorOuttakePID.java
    public double getRawRPM() { return (launcher2.getVelocity() / TICKS_PER_REV) * 60.0; }

    public ShooterState getState() {
        return currentState;
    }

    public boolean getRapidShooterState() {
        return rapidShoot;
    }

    public boolean getServoState() {
        return servoToggle;
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
    public void runLoader() {
        leftLoader.setPower(-1.0);
        rightLoader.setPower(1.0);
    }

    public void stopLoader() {
        leftLoader.setPower(0.0);
        rightLoader.setPower(0.0);
    }

    // ===== STOP =====
    public void stop() {
        stopLoader();
        launcher.setPower(0);
        launcher2.setPower(0);
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
}
