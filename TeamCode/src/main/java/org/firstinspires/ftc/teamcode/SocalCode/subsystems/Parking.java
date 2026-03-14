package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Parking {

    private final Servo parkingServo;

    private static double DRIVE_POSITION = 0.0;
    private static double PARK_POSITION = 1.0;

    private enum State {
        DRIVE,
        PARK
    }

    private State currentState = State.DRIVE;
    private boolean lastButtonState = false;

    public Parking(HardwareMap hardwareMap) {
        parkingServo = hardwareMap.get(Servo.class, "parkingServo");
        parkingServo.setPosition(DRIVE_POSITION);
    }

    public void update(Gamepad gamepad) {
        boolean buttonPressed = gamepad.optionsWasPressed(); // Changed from 'options' to 'start'

        if (buttonPressed && !lastButtonState) {
            toggleState();
        }

        lastButtonState = buttonPressed;
        applyState();

        adjustParkPosition(gamepad);
        adjustDrivePosition(gamepad);
    }

    private void toggleState() {
        switch (currentState) {
            case DRIVE:
                currentState = State.PARK;
                break;
            case PARK:
                currentState = State.DRIVE;
                break;
        }
    }

    private void applyState() {
        switch (currentState) {
            case DRIVE:
                parkingServo.setPosition(DRIVE_POSITION);
                break;
            case PARK:
                parkingServo.setPosition(PARK_POSITION);
                break;
        }
    }

    public boolean isDeployed() {
        return currentState == State.PARK;
    }

    public void adjustDrivePosition(Gamepad gamepad1) {
//        if (gamepad1.dpadUpWasPressed()) {
//            DRIVE_POSITION += 0.1;
//        } else if (gamepad1.dpadDownWasPressed()) {
//            DRIVE_POSITION -= 0.1;
//        }
    }

    public void adjustParkPosition(Gamepad gamepad1) {
//        if (gamepad1.dpadRightWasPressed()) {
//            PARK_POSITION += 0.1;
//        } else if (gamepad1.dpadLeftWasPressed()) {
//            PARK_POSITION -= 0.1;
//        }
    }

    public double getDrivePosition() {
        return DRIVE_POSITION;
    }

    public double getParkPosition() {
        return PARK_POSITION;
    }
}