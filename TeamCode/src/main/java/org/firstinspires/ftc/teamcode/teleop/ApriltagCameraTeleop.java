package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;

@TeleOp(name= "Apriltag camera teleop")
public class ApriltagCameraTeleop extends OpMode {
    private Camera camera;
    private FieldCentricDrive Drive;

    // This constructor is used by the FTC runtime
    public ApriltagCameraTeleop() {
        // The FTC runtime will call init() where the hardware is mapped.
    }

    // This constructor is used for unit testing to inject mock dependencies
    public ApriltagCameraTeleop(Camera camera, FieldCentricDrive drive) {
        this.camera = camera;
        this.Drive = drive;
    }

    @Override
    public void init(){
        // If the camera and drive objects are not already injected (i.e., we are not in a test)
        // then create new instances.
        if (camera == null) {
            camera = new Camera(hardwareMap);
        }
        if (Drive == null) {
            Drive = new FieldCentricDrive(hardwareMap);
        }

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Default to manual drive controls
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Check for alignment override
        if (gamepad1.x) {
            Double turnPower = camera.useBearingToAlign();

            if (turnPower != null) {
                // Alignment successful: override manual controls for turning only
                y = 0; // Stop forward/backward movement
                x = 0; // Stop strafing
                rx = turnPower; // Apply auto-alignment turning power
                telemetry.addData("Status", "Aligning to Tag");
                telemetry.addData("Turn Power", turnPower);
            } else {
                // Alignment requested, but no tag visible: stop all movement
                y = 0;
                x = 0;
                rx = 0;
                telemetry.addData("Status", "No Tag Found");
            }
        } else {
            telemetry.addData("Status", "Manual Control");
        }

        // Apply the final drive powers once at the end of the loop
        Drive.drive(y, x, rx);

        // Handle camera stream and telemetry
        if (gamepad1.dpad_down) {
            camera.stopStream();
        } else if (gamepad1.dpad_up) {
            camera.resumeStream();
        }

        camera.telemetryAprilTag(telemetry);
        telemetry.update();
    }
}
