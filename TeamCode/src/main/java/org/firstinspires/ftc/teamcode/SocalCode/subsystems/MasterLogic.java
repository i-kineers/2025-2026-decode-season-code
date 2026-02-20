package org.firstinspires.ftc.teamcode.SocalCode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * MasterLogic handles the coordination between all subsystems during TeleOp.
 * It encapsulates all the logic so the main OpMode remains clean.
 */
public class MasterLogic {

    private final PanelsTelemetry panelsTelemetry;
    //    private final FlywheelSystem flywheel;
    private final DoubleIntake intake;
    private final Parking parking;
    private final FieldCentricDrive fieldCentricDrive;

    public MasterLogic(HardwareMap hardwareMap, boolean isBlueAlliance) {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        intake = new DoubleIntake(hardwareMap);
        parking = new Parking(hardwareMap);
        fieldCentricDrive = new FieldCentricDrive(hardwareMap);
    }

    public void mainLogic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        // Drive control is now handled entirely by the FieldCentricDrive subsystem
        fieldCentricDrive.update(gamepad1);

        intake.runIntake(gamepad1);
        parking.update(gamepad1);
        updateTelemetry(telemetry);
    }

    private void updateTelemetry(Telemetry telemetry) {
        // Telemetry is simplified as odometry and auto-aim are removed.
        telemetry.addData("Mode", "MANUAL (Field Centric)");
        telemetry.addData("Park Position", parking.getParkPosition());
        telemetry.addData("Drive Position", parking.getDrivePosition());
        // You can add back other subsystem telemetry here if needed
        // e.g., telemetry.addData("Intake State", intake.getState());
        telemetry.update();
    }
}
