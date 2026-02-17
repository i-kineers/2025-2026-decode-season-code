package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Parking {

    private final Servo parkingServo;

    private static final double PARK_POSITION = 0.0;
    private static final double DEPLOY_POSITION = 1.0;

    private enum State {
        PARKED,
        DEPLOYED
    }

    private State currentState = State.PARKED;
    private boolean lastButtonState = false;

    public Parking(HardwareMap hardwareMap) {
        parkingServo = hardwareMap.get(Servo.class, "parkingServo");
        parkingServo.setPosition(PARK_POSITION);
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
            case PARKED:
                currentState = State.DEPLOYED;
                break;
            case DEPLOYED:
                currentState = State.PARKED;
                break;
        }
    }

    private void applyState() {
        switch (currentState) {
            case PARKED:
                parkingServo.setPosition(PARK_POSITION);
                break;
            case DEPLOYED:
                parkingServo.setPosition(DEPLOY_POSITION);
                break;
        }
    }

    public boolean isDeployed() {
        return currentState == State.DEPLOYED;
    }
}