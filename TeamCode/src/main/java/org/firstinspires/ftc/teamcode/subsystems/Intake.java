package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor IntakeMotor;
    Servo GateServo;
    public Intake (HardwareMap hardwareMap){
        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        GateServo = hardwareMap.get(Servo.class, "Gate");
    }

    // Main method
    public void runIntake(double power){
        IntakeMotor.setPower(power);
    }
    public void runGate(double power){ GateServo.setPosition(power); }
    public void stopGate(){
        GateServo.setPosition(0);
    }
    public void stopIntake(){
        IntakeMotor.setPower(0);
    }
    public void stopEntireIntake(){
        IntakeMotor.setPower(0);
        GateServo.setPosition(0);
    }

}
