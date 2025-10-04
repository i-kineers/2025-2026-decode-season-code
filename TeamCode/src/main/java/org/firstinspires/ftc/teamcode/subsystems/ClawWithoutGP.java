package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Claw {
    Servo clawServo;

    double clawPos = 0.5;
    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }
    public void runclaw (Gamepad gamepad2) {
        if (gamepad2.a) {
            clawPos += 0.01;
        } else if (gamepad2.b) {
            clawPos -= 0.01;
        }
        clawPos = Math.max(0, Math.min(1, clawPos));
        clawServo.setPosition(clawPos);

    }
    public double getClawPos(){
        return clawServo.getPosition();
    }
}
