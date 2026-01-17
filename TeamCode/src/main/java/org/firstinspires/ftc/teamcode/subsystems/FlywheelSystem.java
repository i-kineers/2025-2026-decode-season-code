package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

public class FlywheelSystem {

    /* ===================== Hardware ===================== */
    private final DcMotorEx flywheel1;
    private final DcMotorEx flywheel2;
    private final DcMotorEx intake;
    private final VoltageSensor batteryVoltage;

    /* ===================== Tunables ===================== */
    public double kP = 0.001;
    public double kI = 0.0001;
    public double kD = 0.0004;

    public double kF_LOW  = 0.000725;
    public double kF_HIGH = 0.000500;

    public double LOW_TARGET_TPS  = 700; // Old values 1213.0
    public double HIGH_TARGET_TPS = 1260; // Old values 1540.0

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

    private enum ShotState { IDLE, FIRING, RECOVERING }
    private ShotState shotState = ShotState.IDLE;

    /* ===================== Constructor ===================== */
    public FlywheelSystem(HardwareMap hardwareMap) {

        flywheel1 = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        intake    = hardwareMap.get(DcMotorEx.class, "Intake");

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // SAME direction unless your hardware requires inversion
        flywheel1.setDirection(DcMotor.Direction.FORWARD);
        flywheel2.setDirection(DcMotor.Direction.FORWARD);

        pidTimer.reset();
    }

    /* ===================== Public Intent API ===================== */

    public void setTargetTPS(double tps) {
        targetTPS = Math.max(0, tps);
    }

    public void startFiring() {
        if (shotState == ShotState.IDLE) {
            shotsFired = 0;
            shotState = ShotState.FIRING;
        }
    }

    public void stopFiring() {
        shotState = ShotState.IDLE;
    }

    public void stopFlywheel() {
        targetTPS = 0;
        setFlywheelPower(0);
    }

    /* ===================== Main Update (call every loop) ===================== */

    public void update() {
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
        applySafety(finalPower, currentTPS);
        handleShotLogic(targetTPS, currentTPS);
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

    private void handleShotLogic(double target, double current) {
        boolean wheelReady = current >= target * 0.95;

        switch (shotState) {
            case IDLE:
                intake.setPower(0);
                break;

            case FIRING:
                if (wheelReady) {
                    intake.setPower(1.0);
                    if (current < (target * DANGER_THRESHOLD)) {
                        shotsFired++;
                        shotState = ShotState.RECOVERING;
                        shotTimer.reset();
                    }
                }
                break;

            case RECOVERING:
                intake.setPower(0);
                if (wheelReady && shotTimer.milliseconds() > 100) {
                    shotState = (shotsFired >= TARGET_SHOT_COUNT)
                            ? ShotState.IDLE
                            : ShotState.FIRING;
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

    public double getVelocity() {
        return flywheel2.getVelocity();
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
