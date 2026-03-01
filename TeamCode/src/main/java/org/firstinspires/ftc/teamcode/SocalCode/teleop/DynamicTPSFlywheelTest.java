package org.firstinspires.ftc.teamcode.SocalCode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.FlywheelSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "Dynamic TPS Flywheel Test")
public class DynamicTPSFlywheelTest extends LinearOpMode {
    private FlywheelSystem flywheel;
    private Follower follower;
    private DriveSubsystem odometryControl;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    double targetTPS = 1200;
    double idleTPS = 0;

    private static double goalDist;

    private Pose startPose;

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel = new FlywheelSystem(hardwareMap);
        odometryControl = new DriveSubsystem(hardwareMap, true);
        follower = Constants.createFollower(hardwareMap);;

        startPose = new Pose(19.5, 122.6, 135);
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData(">", "Connect to Panels dashboard");
        telemetry.addData(">", "Press A for target RPM, B to stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Increase/Decrease target rpm ---
            if (gamepad1.dpadUpWasPressed()) {
                targetTPS += 10;
            } else if (gamepad1.dpadDownWasPressed()) {
                targetTPS -= 10;
            }

            goalDist = odometryControl.getDistanceFromGoal();
//            targetTPS = odometryControl.newDynamicTargetTPS(goalDist);

            flywheel.cycleShootingState(gamepad1);
            flywheel.update(gamepad1);

            follower.update();
            odometryControl.update();

//            panelsTelemetry.getTelemetry().addData("Target RPM", flywheel.getVelocity());
//            panelsTelemetry.getTelemetry().addData("Current RPM", targetTPS);
//            panelsTelemetry.getTelemetry().update();

            telemetry.addData("Current TPS", flywheel.getVelocity());
            telemetry.addData("Target TPS", targetTPS);
            telemetry.addData("Goal Distance", goalDist);
            telemetry.update();

        }
    }
}
