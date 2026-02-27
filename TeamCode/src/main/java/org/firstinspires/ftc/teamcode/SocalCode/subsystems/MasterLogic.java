package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MasterLogic {

    private final PanelsTelemetry panelsTelemetry;
    private final FlywheelSystem flywheel;
    private final DoubleIntake intake;
    private final Parking parking;
    private final FieldCentricDrive fieldCentricDrive;

    public MasterLogic(HardwareMap hardwareMap, boolean isBlueAlliance) {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        flywheel = new FlywheelSystem(hardwareMap);
        intake = new DoubleIntake(hardwareMap);
        parking = new Parking(hardwareMap);
        fieldCentricDrive = new FieldCentricDrive(hardwareMap);
    }

    public void mainLogic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        fieldCentricDrive.update(gamepad1);

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