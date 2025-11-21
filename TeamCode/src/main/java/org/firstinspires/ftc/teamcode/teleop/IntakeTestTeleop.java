package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
@TeleOp(name = "Intake Test Teleop")
public class IntakeTestTeleop extends OpMode {
    Intake intake;

    public void init(){
        intake = new Intake(hardwareMap);
    }
    @Override
    public void loop(){
        if (gamepad1.x){
            intake.runIntake(1);
        } else {
            intake.runIntake(0);
        }
        if (gamepad1.y){
            intake.runFullIntake(1);
        } else {
            intake.runFullIntake(0);
        }
        telemetry.update();
    }

}
