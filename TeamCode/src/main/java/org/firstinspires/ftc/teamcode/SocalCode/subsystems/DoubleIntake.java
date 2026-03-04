package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DoubleIntake {

    DcMotor leftIntakeMotor;
    DcMotor rightIntakeMotor;

    public enum intakeState {IDLE, INTAKE, OUTTAKE}
    private intakeState currentIntakeState = intakeState.IDLE;

    public enum autoIntakeState {IDLE, INTAKE, SHOOTING}
    private autoIntakeState currentAutoIntakeState = autoIntakeState.IDLE;
    private ElapsedTime autoTimer = new ElapsedTime();

    public DoubleIntake(HardwareMap hardwaremap){
        leftIntakeMotor = hardwaremap.get(DcMotor.class, "leftIntake");
        rightIntakeMotor = hardwaremap.get(DcMotor.class, "rightIntake");
        leftIntakeMotor.setDirection(DcMotor.Direction.REVERSE); // Reverse one of the motors
    }

    public void runIntake(Gamepad gamepad) {
        if (gamepad.right_trigger > 0.1 || gamepad.left_trigger > 0.1) {
            setIntakeState(intakeState.INTAKE);
        } else if (gamepad.right_bumper || gamepad.left_bumper) {
            setIntakeState(intakeState.OUTTAKE);
        } else {
            setIntakeState(intakeState.IDLE);
        }

        intakeFSM(1, gamepad);
    }

    public void intakeFSM(double power, Gamepad gamepad) {
        switch (currentIntakeState) {
            case IDLE:
                setBothIntakePower(0);
                break;

            case INTAKE:
                setLeftIntake(gamepad.left_trigger > 0.1 ? power : 0);
                setRightIntake(gamepad.right_trigger > 0.1 ? power : 0);
                break;

            case OUTTAKE:
                setLeftIntake(gamepad.left_bumper ? -power : 0);
                setRightIntake(gamepad.right_bumper ? -power : 0);
                break;
        }
    }

    public boolean autoIntakeOn(boolean isBlue) {
        switch (currentAutoIntakeState) {
            case IDLE:
                autoIntakeOff();
                return true;

            case INTAKE:
                if (isBlue) {
                    setLeftIntake(1);
                } else {
                    setRightIntake(1);
                }
                return false;

            case SHOOTING: // ADJUST TIMING ACCORDINGLY... BE MINDFUL OF TIME USED SHOOTING!
                if (autoTimer.milliseconds() < 1000) {
                    setLeftIntake(1);
                    setRightIntake(0);
                    return false;
                } else if (autoTimer.milliseconds() < 2000) {
                    setLeftIntake(0);
                    setRightIntake(1);
                    return false;
                } else if (autoTimer.milliseconds() < 2500) {
                    setLeftIntake(1);
                    setRightIntake(1);
                } else {
                    autoIntakeOff();
                    setAutoIntakeState(autoIntakeState.IDLE);
                    return true;
                }

            default:
                return true;
        }
    }

    public void autoIntakeOff() {
        setLeftIntake(0);
        setRightIntake(0);
    }

    public void setLeftIntake(double power) {
        leftIntakeMotor.setPower(power);
    }
    public void setRightIntake(double power) {
        rightIntakeMotor.setPower(power);
    }

    public void setBothIntakePower (double power) {
        setLeftIntake(power);
        setRightIntake(power);
    }

    public void setIntakeState(intakeState state) {
        currentIntakeState = state;
    }

    public void setAutoIntakeState(autoIntakeState autoState) {
        if (autoState == autoIntakeState.SHOOTING && currentAutoIntakeState != autoIntakeState.SHOOTING) {
            autoTimer.reset();
        }
        currentAutoIntakeState = autoState;
    }
}
