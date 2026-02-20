package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Parking {

    private final Servo parkingServo;

    private static final double DRIVE_POSITION = 0.0;
    private static final double PARK_POSITION = 1.0;

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
        boolean buttonPressed = gamepad.x;

        if (buttonPressed && !lastButtonState) {
            toggleState();
        }

        lastButtonState = buttonPressed;
        applyState();
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
}