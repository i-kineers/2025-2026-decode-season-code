package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;

import org.firstinspires.ftc.teamcode.subsystems.BasicOuttake;



@TeleOp(name="Main")
public class MainTeleop extends OpMode {
    private Chassis chassis;
    private BasicOuttake outtake;


    @Override
    public void init() {
        chassis = new Chassis(hardwareMap);
        outtake = new BasicOuttake(hardwareMap);
    }



    @Override
    public void loop() {
        chassis.runMacanumWheels(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        if (gamepad1.a) {
            outtake.setLaunchPower(.5);
            outtake.setSideLaunchPower(1);
        } else if (gamepad1.b) {
            outtake.setLaunchPower(.5);
            outtake.setSideLaunchPower(0);
        } else if (gamepad1.x){
            outtake.emergencyStop();
        }

        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
