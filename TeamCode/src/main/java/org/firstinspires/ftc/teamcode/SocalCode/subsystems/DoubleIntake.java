package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DoubleIntake {

    DcMotor leftIntakeMotor;
    DcMotor rightIntakeMotor;

    public enum intakeState {IDLE, INTAKE, OUTTAKE}
    private intakeState currentIntakeState = intakeState.IDLE;

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
}
