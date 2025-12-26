package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Camera;

@TeleOp(name = "Odo Camera Teleop", group = "TeleOp")
public class OdoCameraTeleop extends OpMode {
    private Camera camera;
    private Follower follower;

    @Override
    public void init() {
        camera = new Camera(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22, 120, 135)); // Initialize starting pose

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        camera.telemetryAprilTag(telemetry);
        
        // Camera Stream Control
        if (gamepad1.dpad_down) {
            camera.stopStream();
        } else if (gamepad1.dpad_up) {
            camera.resumeStream();
        }

        // Drive Control
        if (gamepad1.x) {
            Double turnPower = camera.useBearingToAlign();

            if (turnPower != null) {
                // When aligning, we only control rotation with the camera
                // We stop translational movement (x, y)
                follower.setTeleOpDrive(0, 0, turnPower, false); // false for robot centric if needed, but usually field centric is fine if we just turn
                telemetry.addData("The turn power is = ", turnPower);
            } else {
                // No tag detected — revert to manual control
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            }
        } else {
            // Manual driving with field centric control
            // Note: Gamepad Y is inverted, so we negate it.
            // follower.setTeleOpDrive(forward, strafe, turn, fieldCentric)
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }

        follower.update();

        // Telemetry for Odometry
        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("heading:", follower.getPose().getHeading());
        telemetry.update();
    }
}
