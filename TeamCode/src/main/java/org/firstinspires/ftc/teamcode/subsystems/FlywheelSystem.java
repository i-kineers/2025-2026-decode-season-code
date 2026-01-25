package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

public class FlywheelSystem {

    /* ===================== Hardware ===================== */
    private final DcMotorEx flywheel1;
    private final DcMotorEx flywheel2;
    private final CRServo leftLoader;
    private final CRServo rightLoader;
    private final VoltageSensor batteryVoltage;

    /* ===================== Tunables ===================== */
    public double kP = 0.003;
    public double kI = 0.0; // Old value 0.0001
    public double kD = 0.0; // Old value 0.0004

    public double kF_LOW  = 0.000463;
    public double kF_HIGH = 0.000408;

    public double LOW_TARGET_TPS  = 710; // Old values 1213.0
    public double HIGH_TARGET_TPS = 1800; // Old values 1540.0

    public double MAX_CURRENT = 8.5;
    public double DANGER_THRESHOLD = 0.93;
    public double RECOVERY_SLEW = 1.0;

    public int TARGET_SHOT_COUNT = 3;

    /* ===================== Internal State ===================== */
    private double targetTPS = 0;

    private double integralSum = 0;
    private double lastError = 0;
    private double lastPower = 0;

    private boolean isRecovering = false;
    private double lastRecoveryTime = 0;
    private int shotsFired = 0;

    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();

    private final List<Double> recoveryLog = new ArrayList<>();

    public enum ShotState { IDLE, FIRING, RECOVERING }
    private ShotState shotState = ShotState.IDLE;

    private final ElapsedTime loopTimer;

    /* ===================== Constructor ===================== */
    public FlywheelSystem(HardwareMap hardwareMap) {

        flywheel1 = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        leftLoader = hardwareMap.get(CRServo.class, "leftLoader");
        rightLoader = hardwareMap.get(CRServo.class, "rightLoader");

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // SAME direction unless your hardware requires inversion
        flywheel1.setDirection(DcMotor.Direction.FORWARD);
        flywheel2.setDirection(DcMotor.Direction.FORWARD);

        pidTimer.reset();

        loopTimer = new ElapsedTime();
    }

    /* ===================== Public Intent API ===================== */

    public void setTargetTPS(double tps) {
        targetTPS = Math.max(0, tps);
    }

    public void stopFlywheel() {
        targetTPS = 0;
        setFlywheelPower(0);
    }

    /* ===================== Main Update (call every loop) ===================== */

    public void update() {
        // If targetTPS is 0, we just stop and return.
        // This means MasterLogic MUST set a non-zero targetTPS if it wants it to spin.
        if (targetTPS <= 0) {
            setFlywheelPower(0);
            return;
        }

        double currentTPS = getVelocity();
        double battery = batteryVoltage.getVoltage();

        double rawPower = calculatePIDF(targetTPS, currentTPS);
        double compensated = rawPower * (12.0 / battery);
        double finalPower = applySlew(compensated);

        monitorRecovery(targetTPS, currentTPS);
        handleShotLogic(targetTPS, currentTPS, finalPower);
    }

    /* ===================== PIDF ===================== */

    private double calculatePIDF(double target, double current) {

        double slope = (kF_HIGH - kF_LOW) / (HIGH_TARGET_TPS - LOW_TARGET_TPS);
        double kF = kF_LOW + (target - LOW_TARGET_TPS) * slope;
        double feedforward = target * kF;

        double error = target - current;

        if (current < target * DANGER_THRESHOLD) {
            return 1.0; // Bang-bang recovery
        }

        double dt = Math.max(pidTimer.seconds(), 0.001);

        if (Math.abs(error) < 150) {
            integralSum += error * dt;
        } else {
            integralSum = 0;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;
        pidTimer.reset();

        return feedforward + (kP * error) + (kI * integralSum) + (kD * derivative);
    }

    /* ===================== Shot Awareness ===================== */

    private void monitorRecovery(double target, double current) {
        double ratio = current / target;

        if (!isRecovering && ratio < DANGER_THRESHOLD) {
            isRecovering = true;
            recoveryTimer.reset();
        }

        if (isRecovering && ratio >= 0.98) {
            isRecovering = false;
            lastRecoveryTime = recoveryTimer.milliseconds();
            recoveryLog.add(lastRecoveryTime);
        }
    }

    private void handleShotLogic(double target, double current, double finalPower) {
        boolean wheelReady = current >= target * 0.95;

        switch (shotState) {
            case IDLE:
                applySafety(finalPower, current);
                stopLoader();
                break;

            case FIRING:
                applySafety(finalPower, current);
                if (wheelReady) {
                    if (current < (target * DANGER_THRESHOLD)) {
                        stopLoader();
                        shotsFired++;
//                        shotState = ShotState.RECOVERING;
                        shotTimer.reset();
                    } else {
                        runLoader();
                    }
                }
                break;

            case RECOVERING:
                if (wheelReady && shotTimer.milliseconds() > 100) {
                    shotState = (shotsFired >= TARGET_SHOT_COUNT) ? ShotState.IDLE : ShotState.FIRING;
                }
                break;
        }
    }

    /* ===================== Safety ===================== */

    private void applySafety(double power, double currentTPS) {
        double currentDraw =
                flywheel1.getCurrent(CurrentUnit.AMPS)
                        + flywheel2.getCurrent(CurrentUnit.AMPS);

        if (currentDraw > MAX_CURRENT && currentTPS < 100) {
            setFlywheelPower(0);
        } else {
            setFlywheelPower(Range.clip(power, 0, 1));
        }
    }

    /* ===================== Helpers ===================== */

    private void setFlywheelPower(double power) {
        flywheel1.setPower(power);
        flywheel2.setPower(power);
    }

    private double applySlew(double targetPower) {
        double delta = targetPower - lastPower;
        if (Math.abs(delta) > RECOVERY_SLEW) {
            targetPower = lastPower + Math.signum(delta) * RECOVERY_SLEW;
        }
        lastPower = targetPower;
        return targetPower;
    }

    public void runLoader() {
        leftLoader.setPower(-1.0);
        rightLoader.setPower(1.0);
    }

    public void stopLoader() {
        leftLoader.setPower(0.0);
        rightLoader.setPower(0.0);
    }

    public double getVelocity() {
        return flywheel2.getVelocity();
    }

    public void autoRapidShoot(double tps, long time, double delay) {
        loopTimer.reset();
        setTargetTPS(tps);

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

    public void sleep(long milli) {
        try {
            Thread.sleep(milli);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void stop() {
        stopLoader();
        setFlywheelPower(0);
        targetTPS = 0;
    }

    public void handleTriggerInput(double triggerValue, double tps, double idleTps) {
        if (triggerValue > 0.1) { // If trigger is held down
            setTargetTPS(tps);
            setShotStateFire();
        } else {
            // When released, stop firing and spin down
            setTargetTPS(idleTps);
            setShotStateIdle();
        }
    }

    public void setShotStateIdle() {
        shotState = ShotState.IDLE;
    }

    public void setShotStateFire() {
        shotState = ShotState.FIRING;
    }

    /* ===================== Read-only Accessors ===================== */

    public double getLastRecoveryTime() {
        return lastRecoveryTime;
    }

    public ShotState getShotState() {
        return shotState;
    }

    public List<Double> getRecoveryLog() {
        return recoveryLog;
    }
}
