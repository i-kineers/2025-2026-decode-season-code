package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor IntakeMotor;
    Servo GateServo;
    public Intake (HardwareMap hardwareMap){
        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake");
    }

    // Main method
    public void runIntake(double power){
        IntakeMotor.setPower(power);
    }
}
