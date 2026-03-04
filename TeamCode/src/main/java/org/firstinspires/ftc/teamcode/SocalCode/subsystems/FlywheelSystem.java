package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

public class FlywheelSystem {

    private final DcMotorEx flywheel;
    private final DcMotorEx flywheel2;
    private final Servo legKicker;
    private final Servo hoodServo;
    private final CRServo wheelKicker;
    private final VoltageSensor batteryVoltage;

    public double kP = 0.0035;
    public double kI = 0.0; // Old value 0.0001
    public double kD = 0.0; // Old value 0.0004

    public double kF_LOW  = 0.000490;
    public double kF_HIGH = 0.000426;

    public double LOW_TARGET_TPS  = 700;
    public double HIGH_TARGET_TPS = 1780;

    public double DANGER_THRESHOLD = 0.93;
    public double RECOVERY_SLEW = 1.0;

    private double normalTPS = 1100;
    private double idleTPS = 1100;
    private double hoodPos = 1;
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
    public enum ShotState { OFF, IDLE, FIRING }
    private ShotState shotState = ShotState.OFF;
    public enum ShooterState {OFF, SHOOTING, INTAKING}
    private ShooterState shooterState = ShooterState.INTAKING;

    private boolean autoShootStarted = false;

    public FlywheelSystem(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        legKicker = hardwareMap.get(Servo.class, "legKicker");
        wheelKicker = hardwareMap.get(CRServo.class, "wheelKicker");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel2.setDirection(DcMotor.Direction.REVERSE);

        pidTimer.reset();
    }

    public void update(Gamepad gamepad) {
        if (targetTPS <= 0) {
            setFlywheelPower(0);
            return;
        }

        double finalPower = calculateCompensatedPower();

        monitorRecovery();
        handleShotLogic(finalPower, gamepad);
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

    private void handleShotLogic(double finalPower, Gamepad gamepad) {
        switch (shooterState)  {
            case OFF:
                setFlywheelPower(0);
                legKicker.setPosition(1);
                wheelKicker.setPower(0);
                break;
            case INTAKING:
                setFlywheelPower(idleTPS);
                if (gamepad.right_trigger > 0.5 || gamepad.left_trigger > 0.5) {
                    wheelKicker.setPower(1);
                    legKicker.setPosition(1);
                } else {
                    wheelKicker.setPower(0);
                    legKicker.setPosition(1);
                }
                break;
            case SHOOTING:
                setFlywheelPower(finalPower);
                if (gamepad.right_trigger > 0.5 || gamepad.left_trigger > 0.5) {
                    wheelKicker.setPower(-1);
                    legKicker.setPosition(0);
                } else {
                    wheelKicker.setPower(0);
                    legKicker.setPosition(1);
                }
                break;
        }
    }

    public void cycleShootingState(Gamepad gamepad, Gamepad gamepad2) {
        if (gamepad.yWasPressed()) {
            if (shooterState == ShooterState.INTAKING) {
                shooterState = ShooterState.SHOOTING;
            } else if (shooterState == ShooterState.SHOOTING || shooterState == ShooterState.OFF) {
                shooterState = ShooterState.INTAKING;
            }
        }

        if (gamepad2.dpadUpWasPressed()) {
            hoodPos += 0.1;
        } else if (gamepad2.dpadDownWasPressed()) {
            hoodPos -= 0.1;
        }
        hoodControl(hoodPos);

        if (gamepad.startWasPressed()) {
            shooterState = ShooterState.OFF;
        }
    }

    public void autoShootLogic(boolean kickers) {
        if (targetTPS <= 0) {
            setFlywheelPower(0);
        }

        double finalPower = calculateCompensatedPower();

        autoIdleFlywheel(finalPower, kickers);
    }

    public void autoShoot(double finalPower) {
        setFlywheelPower(finalPower);

        // Only reset the timer the very first time this is called
        if (!autoShootStarted) {
            shotTimer.reset();
            autoShootStarted = true;
        }

        double currentTime = shotTimer.milliseconds();

        if (currentTime > 200 && currentTime <= 1600) {
            wheelKicker.setPower(-1);
            legKicker.setPosition(0);
        } else if (currentTime > 1600) {
            wheelKicker.setPower(0);
            legKicker.setPosition(1);
            autoShootStarted = false; // Reset the flag for the next shot
        }
    }

    public void autoIdleFlywheel(double finalPower, boolean runKickers) {
        setFlywheelPower(finalPower);

        if (runKickers) {
            wheelKicker.setPower(-1);
            legKicker.setPosition(0);
        } else {
            legKicker.setPosition(1);
            wheelKicker.setPower(1);
        }
    }

    public void setKickerState(boolean on) {

    }

    public void sleep(long milli) {
        try {
            Thread.sleep(milli);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void setFlywheelPower(double power) {
        power = Range.clip(power, 0, 1);
        flywheel.setPower(power);
        flywheel2.setPower(power);
    }

    public void hoodControl(double position) {
        hoodServo.setPosition(position);
    }

    private void runWheelServo(double position) {
        wheelKicker.setPower(position);
    }

    private void runLegServo(double position) {
        legKicker.setPosition(position);
    }


    public void setTargetTPS(double tps) { targetTPS = Math.max(0, tps); }

    public double getTargetTPS() {return targetTPS;}

    public void setFlywheelState(ShotState state) {
        shotState = state;
    }

    public void setNormalTPS(double tps) { this.normalTPS = tps; }

    public double getHoodPos() { return hoodPos; }

    public double getLastRecoveryTime() {
        return lastRecoveryTime;
    }

    public double getVelocity() {
        return flywheel.getVelocity();
    }

    public ShotState getShotState() {
        return shotState;
    }

    public ShooterState getShooterState() { return shooterState; }
}
