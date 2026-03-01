package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MasterLogic {

    private final PanelsTelemetry panelsTelemetry;
    private final DriveSubsystem odometry;
    private final FlywheelSystem flywheel;
    private final DoubleIntake intake;
    private final Parking parking;

    public MasterLogic(HardwareMap hardwareMap, boolean isBlueAlliance) {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        flywheel = new FlywheelSystem(hardwareMap);
        intake = new DoubleIntake(hardwareMap);
        parking = new Parking(hardwareMap);
        odometry = new DriveSubsystem(hardwareMap, isBlueAlliance);

        odometry.setStartingPose(19.5, 122.6, 135);
    }

    public void mainLogic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        // Update Localization
        odometry.update();

        // Drive Controls
        odometry.drive(
                -gamepad1.left_stick_y, // Forward
                -gamepad1.left_stick_x, // Strafe
                -gamepad1.right_stick_x, // Turn
                gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_down, gamepad1.dpad_left, // Pathing
                gamepad1.right_bumper // Auto Aim
        );

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