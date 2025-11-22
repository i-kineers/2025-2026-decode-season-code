package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
@TeleOp(name = "Intake Test Teleop")
public class IntakeTestTeleop extends OpMode {
    Intake intake;
    private double power = 0;

    public void init(){
        intake = new Intake(hardwareMap);
    }
    @Override
    public void loop(){
        if (gamepad1.right_trigger == 1){
            intake.runIntake(power);
            // turns on intake
        } else if (gamepad1.aWasPressed()) {
            power += 0.1;
        } else if (gamepad1.bWasPressed()) {
            power -= 0.1;
        } else {
            intake.runIntake(0);
            // intake not spinning and gate closed
        }

        telemetry.addData("Power", power);
        telemetry.update();
    }

}
