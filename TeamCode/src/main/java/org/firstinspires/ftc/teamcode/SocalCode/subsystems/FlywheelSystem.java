package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

public class FlywheelSystem {

    private final DcMotorEx flywheel;
    private final DcMotorEx kicker;
    private final VoltageSensor batteryVoltage;

    public double kP = 0.003;
    public double kI = 0.0; // Old value 0.0001
    public double kD = 0.0; // Old value 0.0004

    public double kF_LOW  = 0.000463;
    public double kF_HIGH = 0.000408;

    public double LOW_TARGET_TPS  = 710; // Old values 1213.0
    public double HIGH_TARGET_TPS = 1800; // Old values 1540.0

    public double DANGER_THRESHOLD = 0.93;
    public double RECOVERY_SLEW = 1.0;

    private double normalTPS = 1213;
    private double idleTPS = 1000;
    private double targetTPS = normalTPS;

    private boolean idleMode = true;

    private double integralSum = 0;
    private double lastError = 0;
    private double lastPower = 0;

    private boolean isRecovering = false;
    private double lastRecoveryTime = 0;

    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();

    private final List<Double> recoveryLog = new ArrayList<>();

    public enum ShotState { OFF, IDLE, FIRING}
    private ShotState shotState = ShotState.OFF;

    private final ElapsedTime loopTimer;

    public FlywheelSystem(HardwareMap hardwareMap) {

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        kicker = hardwareMap.get(DcMotorEx.class, "kicker");

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel.setDirection(DcMotor.Direction.FORWARD);

        pidTimer.reset();

        loopTimer = new ElapsedTime();
    }

    public void runFlywheel(Gamepad gamepad1, Gamepad gamepad2) {
        update();

        if (gamepad1.a) {
            setFlywheelState(ShotState.FIRING);
        } else if (gamepad1.startWasPressed()) {
            setFlywheelState(ShotState.OFF);
            idleMode = false;
        } else if (idleMode) {
            setFlywheelState(ShotState.IDLE);
        }

        if (gamepad2.dpadUpWasPressed()) {
            normalTPS += 50;
        } else if (gamepad2.dpadDownWasPressed()) {
            normalTPS  -= 50;
        }
    }

    public void update() {
        if (targetTPS < 0) {
            setFlywheelPower(0);
            return;
        }

        double finalPower = calculateCompensatedPower();

        monitorRecovery();
        handleShotLogic(finalPower);
    }

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

    // Shot awareness LOGGER function
    private void monitorRecovery() {
        double ratio = getVelocity() / targetTPS;

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

    private double applySlew(double targetPower) {
        double delta = targetPower - lastPower;
        if (Math.abs(delta) > RECOVERY_SLEW) {
            targetPower = lastPower + Math.signum(delta) * RECOVERY_SLEW;
        }
        lastPower = targetPower;
        return targetPower;
    }

    private double calculateCompensatedPower() {
        double currentTPS = getVelocity();
        double battery = batteryVoltage.getVoltage();

        double rawPower = calculatePIDF(targetTPS, currentTPS);
        double compensated = rawPower * (12.0 / battery);

        return applySlew(compensated);
    }

    private void handleShotLogic(double finalPower) {
        boolean wheelReady = getVelocity() >= targetTPS * 0.95;

        switch (shotState) {
            case OFF:
                setTargetTPS(0);
                break;

            case IDLE:
                setTargetTPS(idleTPS);
                setFlywheelPower(Range.clip(finalPower, 0, 1));
                stopKicker();
                break;

            case FIRING:
                setTargetTPS(normalTPS);
                setFlywheelPower(Range.clip(finalPower, 0, 1));
                if (wheelReady) {
                    if (getVelocity() < (targetTPS * DANGER_THRESHOLD)) {
                        stopKicker();
                        shotTimer.reset();
                    } else {
                        runKicker();
                    }
                }
                break;
        }
    }

    public void autoRapidShoot(double tps, long time, double delay) {
        loopTimer.reset();
        setTargetTPS(tps);

        ElapsedTime timer = new ElapsedTime();

        while (timer.milliseconds() < time) {
            if (loopTimer.milliseconds() > delay) {
                runKicker();
            }
            update();
            sleep(10);
        }
        stop();
        update();
    }

    public void sleep(long milli) {
        try {
            Thread.sleep(milli);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void stop() {
        stopKicker();
        setFlywheelPower(0);
        targetTPS = 0;
    }

    private void setFlywheelPower(double power) {
        flywheel.setPower(power);
    }

    public void setTargetTPS(double tps) {
        targetTPS = Math.max(0, tps);
    }

    public void runKicker() {
        kicker.setPower(1);
    }

    public void stopKicker() {
        kicker.setPower(0);
    }

    public void setFlywheelState(ShotState state) {
        shotState = state;
    }

    public double getLastRecoveryTime() {
        return lastRecoveryTime;
    }

    public double getVelocity() {
        return flywheel.getVelocity();
    }

    public ShotState getShotState() {
        return shotState;
    }

    public List<Double> getRecoveryLog() {
        return recoveryLog;
    }
}
