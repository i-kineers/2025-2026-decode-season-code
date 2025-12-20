package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
@TeleOp(name= "Apriltag camera teleop")
public class ApriltagCameraTeleop extends OpMode {
    private Camera camera;
    private FieldCentricDrive Drive;

    public void init(){
        camera = new Camera(hardwareMap);
        Drive = new FieldCentricDrive(hardwareMap);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
    }
    @Override
    public void loop(){
        camera.telemetryAprilTag(telemetry);
        telemetry.update();
        if (gamepad1.dpad_down) {
            camera.stopStream();
        } else if (gamepad1.dpad_up) {
            camera.resumeStream();
        }
        if (gamepad1.x){
            Double turnPower = camera.useBearingToAlign();

            if (turnPower != null) {

                Drive.drive(0, 0, turnPower);
                telemetry.addData("The turn power is = ", turnPower);

            }
            else {
                // No tag detected — stop or do something else
                Drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

        } else {
            Drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }


    }
}
