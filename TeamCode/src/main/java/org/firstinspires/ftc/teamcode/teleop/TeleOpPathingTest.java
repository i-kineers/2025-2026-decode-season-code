package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.Pose;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.Paths.teleopPath;
import org.firstinspires.ftc.teamcode.autonomous.Paths.testPathCloseBlue;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Camera;

import static android.os.SystemClock.sleep;

@TeleOp(name = "TeleOp Pathing & PID Test", group = "Test")
public class TeleOpPathingTest extends LinearOpMode {

    private Camera camera;
    private Follower follower;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    // Define a target pose for the "Go To Point" feature
    private final Pose targetPose = new Pose(35, 35, Math.toRadians(90));

    private teleopPath paths;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Subsystems
        follower = Constants.createFollower(hardwareMap);

        // Set a starting pose (update this to match your actual start if needed)
        follower.setStartingPose(new Pose(22, 120, 135));

        double targetPosX = 48;
        double targetPosY = 95;

        paths = new teleopPath(follower, targetPose.getX(), targetPose.getY(), targetPosX, targetPosY);
        waitForStart();

        follower.startTeleopDrive();

        while (opModeIsActive()) {
            // Update Follower (crucial for odometry and pathing)
            follower.update();

            // --- Pathing Logic ---
            // Press B to move to the target pose
            if (gamepad1.b) {
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.Path3);
                    setPathState(4);
                }
            }

            // --- Manual Drive (Optional) ---
            // If not following a path, allow manual control
            if (!follower.isBusy()) {
                follower.setTeleOpMovementVectors(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false // robotCentric = false (field centric)
                );
            }
            
            panelsTelemetry.getTelemetry().addData("Robot X", follower.getPose().getX());
            panelsTelemetry.getTelemetry().addData("Robot Y", follower.getPose().getY());
            panelsTelemetry.getTelemetry().addData("Robot H", Math.toDegrees(follower.getPose().getHeading()));
            
            panelsTelemetry.getTelemetry().update();
            
            // Also update standard telemetry for driver station
            telemetry.addData("Mode", follower.isBusy() ? "Pathing" : "Manual");
            telemetry.update();
        }
    }
}
