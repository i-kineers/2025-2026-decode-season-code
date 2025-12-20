package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
@TeleOp(name= "Apriltag camera teleop")
public class ApriltagCameraTeleop extends OpMode {
    private Camera camera;
    private FieldCentricDrive Drive;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

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
        if (gamepad1.dpad_down) {
            camera.stopStream();
        } else if (gamepad1.dpad_up) {
            camera.resumeStream();
        }
        if (gamepad1.right_trigger == 1) {
            camera.increaseP();
        } else if (gamepad1.left_trigger == 1) {
            camera.decreaseP();
        }

        if (gamepad1.right_bumper) {
            camera.increaseD();
        } else if (gamepad1.left_bumper) {
            camera.decreaseD();
        }
        // Add I tuning (Left/Right D-Pad for I)
        else if (gamepad1.dpad_right) {
            camera.increaseI();
        } else if (gamepad1.dpad_left) {
            camera.decreaseI();
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

        panelsTelemetry.getTelemetry().addData("kP", camera.getP());
        panelsTelemetry.getTelemetry().addData("kI", camera.getI());
        panelsTelemetry.getTelemetry().addData("kD", camera.getD());
        panelsTelemetry.getTelemetry().update();

        telemetry.addData("P value:", camera.getP());
        telemetry.addData("I value:", camera.getI());
        telemetry.addData("D value:", camera.getD());
        telemetry.update();
    }
}
