package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;

@TeleOp(name="Main")
public class Main extends OpMode {
    private Chassis chassis;

    @Override
    public void init() {
        chassis = new Chassis(hardwareMap);
    }

    @Override
    public void loop() {
        chassis.runMacanumWheels(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

//    @Override
//    public void runOpMode() throws InterruptedException {
//        chassis = new Chassis(hardwareMap);
//    }
}
