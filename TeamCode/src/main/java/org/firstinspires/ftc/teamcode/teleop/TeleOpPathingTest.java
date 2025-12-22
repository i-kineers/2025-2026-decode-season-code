package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.Paths.teleopPath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;

@TeleOp(name = "TeleOp Pathing & PID Test", group = "Test")
public class TeleOpPathingTest extends OpMode {

    private FieldCentricDrive chassis;
    private Camera camera;
    private Follower follower;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private final Pose targetPose = new Pose(35, 35, Math.toRadians(90));
    private teleopPath path;

    /**
     * This runs once when you press "INIT"
     */
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22, 120, 135));

        double targetPosX = 48;
        double targetPosY = 95;
        double headingPos = 135;

        path = new teleopPath(follower, follower.getPose().getX(), follower.getPose().getY(), targetPosX, targetPosY, headingPos);
    }

    /**
     * This runs once when you press "PLAY"
     */
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /**
     * This runs repeatedly after "PLAY" is pressed until "STOP" is pressed
     */
    @Override
    public void loop() {
        follower.update();

        // Path following trigger
        if (gamepad1.b && !follower.isBusy()) {
            follower.setMaxPower(1);
            follower.followPath(path.Path1);
        }

        // Manual TeleOp control
        if (!follower.isBusy()) {
            follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    false
            );
        }

//        if (!follower.isBusy()) {
//            chassis.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        }

        if (gamepad1.a) { // Check for a *press* event
            chassis.resetIMU();
            telemetry.addLine("IMU Reset.");
        }

        // Telemetry Updates
        panelsTelemetry.getTelemetry().addData("Robot X", follower.getPose().getX());
        panelsTelemetry.getTelemetry().addData("Robot Y", follower.getPose().getY());
        panelsTelemetry.getTelemetry().addData("Robot H", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.getTelemetry().update();

        telemetry.addData("Mode", follower.isBusy() ? "Pathing" : "Manual");
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.update();
    }
}
