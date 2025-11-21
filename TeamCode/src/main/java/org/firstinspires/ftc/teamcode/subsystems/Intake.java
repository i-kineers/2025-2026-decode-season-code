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
    public void runIntake(double power){
        IntakeMotor.setPower(power);
    }
    public void runFullIntake(double Powers, double GatePosition){
        IntakeMotor.setPower(Powers);
        GateServo.setPosition(GatePosition);
    }
}
