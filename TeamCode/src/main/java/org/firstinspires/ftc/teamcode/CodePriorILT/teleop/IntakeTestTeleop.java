package org.firstinspires.ftc.teamcode.CodePriorILT.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CodePriorILT.subsystems.Intake;
@TeleOp(name = "Intake Test Teleop")
public class IntakeTestTeleop extends OpMode {
    Intake intake;
    private double power = 0;
    private double doorPosition = 0;

    public void init(){
        intake = new Intake(hardwareMap);
    }
    @Override
    public void loop(){
        if (gamepad1.right_trigger == 1){
            intake.runIntake(-power);
        } else if (gamepad1.right_bumper) {
            intake.runIntake(power);
        }
        else if (gamepad1.left_trigger == 1) {
            intake.runGate(doorPosition);
        } else if (gamepad1.aWasPressed()) {
            power += 0.1;
        } else if (gamepad1.bWasPressed()) {
            power -= 0.1;
        } else if (gamepad1.xWasPressed()) {
            doorPosition += 0.1;
        } else if (gamepad1.yWasPressed()) {
            doorPosition -= 0.1;
        } else {
            intake.stopEntireIntake();
            // intake not spinning and gate closed
        }

        telemetry.addData("Power", power);
        telemetry.addData("Gate Position", doorPosition);
        telemetry.update();
    }

}
