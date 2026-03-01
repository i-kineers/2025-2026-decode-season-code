package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MasterLogic {

    private final PanelsTelemetry panelsTelemetry;
    private final AutoAimWithOdometry odometry;
    private final FlywheelSystem flywheel;
    private final DoubleIntake intake;
    private final Parking parking;

    public MasterLogic(HardwareMap hardwareMap, boolean isBlueAlliance) {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        flywheel = new FlywheelSystem(hardwareMap);
        intake = new DoubleIntake(hardwareMap);
        parking = new Parking(hardwareMap);
        odometry = new AutoAimWithOdometry(hardwareMap, true);

        odometry.setStartingPose(19.5, 122.6, 135);
    }

    public void mainLogic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {

        // 1️⃣ ALWAYS update the follower first
        odometry.update();

        // 2️⃣ Read sticks. Most drive methods (including Pedro's) expect raw joystick values
        // where Y is negative up. They handle the negation internally.
        // We only negate turn to make CCW positive, which is a standard convention.
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // 3️⃣ Auto pathing triggers (D-pad paths)
        odometry.handlePathing(
                gamepad1.dpad_up,
                gamepad1.dpad_right,
                gamepad1.dpad_down,
                gamepad1.dpad_left
        );

        // 4️⃣ Check for manual override to cancel D-pad paths
        boolean manualInput = Math.abs(forward) > 0.1 ||
                Math.abs(strafe) > 0.1 ||
                Math.abs(turn) > 0.1;
        odometry.handlePathCancel(manualInput);

        // 5️⃣ Drive Control Flow
        if (odometry.isAutomated()) {
            // Robot is following a path (D-pad), do nothing here
            telemetry.addData("Status", "Executing Auto Path...");
        } else {
            // 6️⃣ Manual / Auto-Aim Combined Drive
            if (gamepad1.b) {
                // --- AUTO AIM MODE ---
                double autoAimTurn = odometry.aimWhileMoving();

                // Drive using Follower with Auto-Aim turn
                odometry.drive(forward, strafe, autoAimTurn);
                telemetry.addData("Status", "AUTO-AIM ACTIVE");
            } else {
                // --- MANUAL MODE ---
                // Drive using Follower with Manual turn
                odometry.drive(forward, strafe, turn);
                telemetry.addData("Status", "Manual Drive");
            }
        }

        // 7️⃣ Reset Aim Goal
        if (gamepad1.y) odometry.resetAim();

        // 8️⃣ Reset Heading (Replaces FieldCentricDrive's reset)
        if (gamepad1.a) odometry.resetHeading();

        // Subsystems
        intake.runIntake(gamepad1);
        parking.update(gamepad1);
        flywheel.cycleShootingState(gamepad1);
        flywheel.update(gamepad1);

        updateTelemetry(telemetry);
    }

    private void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Mode", "MANUAL (Field Centric)");
        telemetry.addData("Shooter State", flywheel.getShotState());
        telemetry.addData("Flywheel Velocity", flywheel.getVelocity());
        telemetry.addData("Park Position", parking.getParkPosition());
        telemetry.addData("Drive Position", parking.getDrivePosition());
        telemetry.update();
    }
}