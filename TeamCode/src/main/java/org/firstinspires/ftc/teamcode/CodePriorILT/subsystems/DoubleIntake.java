package org.firstinspires.ftc.teamcode.CodePriorILT.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class DoubleIntake {

    DcMotor frontIntakeMotor;
    DcMotor backIntakeMotor;

    public DoubleIntake(){
        frontIntakeMotor = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntakeMotor = hardwareMap.get(DcMotor.class, "backIntake");
        frontIntakeMotor.setDirection(DcMotor.Direction.REVERSE); // Reverse one of the motors
    }

    public void frontIntake(int power) {
        frontIntakeMotor.setPower(power);
    }
    public void backIntake(int power) {
        backIntakeMotor.setPower(power);
    }
}
