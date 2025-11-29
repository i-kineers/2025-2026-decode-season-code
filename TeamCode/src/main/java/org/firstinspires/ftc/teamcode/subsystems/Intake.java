package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    DcMotor IntakeMotor;
    Servo GateServo;
    private final ElapsedTime loopTimer;

    public Intake (HardwareMap hardwareMap){
        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        GateServo = hardwareMap.get(Servo.class, "Gate");
        loopTimer = new ElapsedTime();
    }

    // Main method
    public void runIntake(double power){
        IntakeMotor.setPower(power);
    }
    public void runGate(double power){ GateServo.setPosition(power); }
    public void stopGate(){
        GateServo.setPosition(-1);
    }
    public void stopIntake(){
        IntakeMotor.setPower(0);
    }
    public void stopEntireIntake(){
        IntakeMotor.setPower(0);
        GateServo.setPosition(-1);
    }

    public void autoIntakeOn() {
        runIntake(-1);
        runGate(1);
    }

    public void autoIntakeOff() {
        runIntake(0);
        runGate(0);
    }
}
