package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TeleOpPathingManager;

@TeleOp(name = "TeleOp Pathing", group = "Test")
public class TeleOpPathingTest extends OpMode {

    private TeleOpPathingManager pathingManager;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    @Override
    public void init() {
        pathingManager = new TeleOpPathingManager(hardwareMap, new Pose(22, 120, Math.toRadians(135)));
    }

    @Override
    public void start() {
        // Start is handled in the manager's constructor
    }

    @Override
    public void loop() {
        pathingManager.update();
        pathingManager.drive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                gamepad1.a
        );

        // Telemetry
        Pose currentPose = pathingManager.getFollower().getPose();
        if (currentPose != null) {
            panelsTelemetry.getTelemetry().addData("Robot X", currentPose.getX());
            panelsTelemetry.getTelemetry().addData("Robot Y", currentPose.getY());
            panelsTelemetry.getTelemetry().addData("Robot H", Math.toDegrees(currentPose.getHeading()));
            panelsTelemetry.getTelemetry().update();

            telemetry.addData("Mode", pathingManager.isAutomated() ? "Pathing" : "Manual");
            telemetry.addData("Robot X", currentPose.getX());
            telemetry.addData("Robot Y", currentPose.getY());
            telemetry.addData("Robot H", Math.toDegrees(currentPose.getHeading()));
        }
        telemetry.update();
    }
}
