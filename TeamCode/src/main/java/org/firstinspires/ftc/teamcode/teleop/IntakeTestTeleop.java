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
        if (gamepad1.right_trigger == 1){
            intake.runIntake(1);
            // turns on intake
        } else {
            intake.runIntake(0);
            // intake not spinning and gate closed
        }
        telemetry.update();
    }

}
