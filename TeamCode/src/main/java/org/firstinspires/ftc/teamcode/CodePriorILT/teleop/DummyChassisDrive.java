package org.firstinspires.ftc.teamcode.CodePriorILT.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.CodePriorILT.subsystems.Chassis;

@TeleOp(name = "Dummy Chassis Drive")
public class DummyChassisDrive extends LinearOpMode {
    private Chassis chassis;

    @Override
    public void runOpMode() throws InterruptedException {
        
        // Initialize the chassis subsystem
        chassis = new Chassis(hardwareMap);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        while (opModeIsActive()) {
            // Pass gamepad inputs to the chassis subsystem
            chassis.runMacanumWheels(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}
