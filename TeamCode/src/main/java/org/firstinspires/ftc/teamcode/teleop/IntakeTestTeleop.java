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
            // turns on intake
        } else if( gamepad1.y){
            intake.runFullIntake(1, 1);
            // takes all balls and keeps from the outtake
        } else if(gamepad1.b){
            intake.runFullIntake(-1, 1);
            // removes balls keeps the gate engage not allow in outtake.
        } else if(gamepad1.a){
            intake.runFullIntake(-1, -1);
            // closes ball hole and allow them to go in outtake
        } else {
            intake.runFullIntake(0, -1);
            // intake not spinning and gate closed
        }
        telemetry.update();
    }

}
