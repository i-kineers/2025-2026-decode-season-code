package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor IntakeMotor;
    CRServo GateServo;
    public Intake (HardwareMap hardwareMap){
        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        GateServo = hardwareMap.get(CRServo.class, "Gate");
    }
    public void runIntake(double power){
        IntakeMotor.setPower(power);
    }
    public void runFullIntake(double Powers){
        IntakeMotor.setPower(Powers);
        GateServo.setPower(Powers);
    }
}
