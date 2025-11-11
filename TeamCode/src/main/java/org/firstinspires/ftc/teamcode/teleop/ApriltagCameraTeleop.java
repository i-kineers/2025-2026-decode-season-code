package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Camera;
@TeleOp(name= "Apriltag camera teleop")
public class ApriltagCameraTeleop extends OpMode {
    private Camera camera;

    public void init(){
        camera = new Camera(hardwareMap);
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
    }
}
