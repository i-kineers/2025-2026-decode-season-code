package org.firstinspires.ftc.teamcode.SocalCode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SocalCode.subsystems.DoubleIntake;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.FlywheelSystem;


@TeleOp(name = "Dynamic TPS Flywheel Test")
public class DynamicTPSFlywheelTest extends LinearOpMode {
    private FlywheelSystem flywheel;
    private DoubleIntake intake;
    private DriveSubsystem odometryControl;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    double targetTPS = 1200;

    private static double goalDist;

    // D-pad state tracking
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel = new FlywheelSystem(hardwareMap);
        intake = new DoubleIntake(hardwareMap);
        odometryControl = new DriveSubsystem(hardwareMap, true);

        // Set the starting pose on the DriveSubsystem, which handles the follower
        odometryControl.setStartingPose(19.5, 122.6, 135);

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData(">", "Connect to Panels dashboard");
        telemetry.addData(">", "Press A for target RPM, B to stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update all subsystems
            odometryControl.update();

            // --- Increase/Decrease target rpm ---
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;

            if (currentDpadUp && !lastDpadUp) {
                targetTPS += 10;
            } else if (currentDpadDown && !lastDpadDown) {
                targetTPS -= 10;
            }
            flywheel.setTargetTPS(targetTPS);


            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;

            goalDist = odometryControl.getDistanceFromGoal();
//            targetTPS = odometryControl.newDynamicTargetTPS(goalDist);

            // Run other subsystems
            intake.runIntake(gamepad1);
            flywheel.cycleShootingState(gamepad1, gamepad2);
            flywheel.update(gamepad1);

//            panelsTelemetry.getTelemetry().addData("Target RPM", flywheel.getVelocity());
//            panelsTelemetry.getTelemetry().addData("Current RPM", targetTPS);
//            panelsTelemetry.getTelemetry().update();

            telemetry.addData("Current TPS", flywheel.getVelocity());
            telemetry.addData("Target TPS", targetTPS);
            telemetry.addData("Goal Distance", goalDist);
            telemetry.addData("Robot X", odometryControl.getRobotX());
            telemetry.addData("Robot Y", odometryControl.getRobotY());
            telemetry.update();

        }
    }
}
