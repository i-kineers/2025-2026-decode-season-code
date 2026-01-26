package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DoubleIntake {

    DcMotor leftIntakeMotor;
    DcMotor rightIntakeMotor;

    public enum intakeState {IDLE, INTAKE, OUTTAKE}
    private intakeState currentIntakeState = intakeState.IDLE;

    private boolean leftIntake = false;
    private boolean rightIntake = false;
    private boolean leftOuttake = false;
    private boolean rightOuttake = false;

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

        intakeFSM(1);
    }

    public void intakeFSM(double power) {
        switch (currentIntakeState) {
            case IDLE:
                setBothIntakePower(0);
                break;
            case INTAKE:
                if (leftIntake) { setLeftIntake(power); }
                if (rightIntake) {setRightIntake(power); }
                break;
            case OUTTAKE:
                if (leftOuttake) { setLeftIntake(-power); }
                if (rightOuttake) { setRightIntake(-power); }
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
