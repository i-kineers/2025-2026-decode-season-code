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

    private double targetTPS;

    private double goalDist;
//    private final Parking parking;

    public MasterLogic(HardwareMap hardwareMap, double startingX, double startingY, double startingH, boolean isBlueAlliance) {
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        flywheel = new FlywheelSystem(hardwareMap);
        intake = new DoubleIntake(hardwareMap);
//        parking = new Parking(hardwareMap);
        odometry = new DriveSubsystem(hardwareMap, isBlueAlliance);

        odometry.setStartingPose(startingX,startingY,startingH);
    }

    public void mainLogic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        // Update Localization
        odometry.update();

        // Drive Controls
        odometry.drive(
                gamepad1.left_stick_y, // Forward
                gamepad1.left_stick_x, // Strafe
                -gamepad1.right_stick_x, // Turn
                gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_down, gamepad1.dpad_left, // Pathing
                gamepad1.b // Auto Aim
        );


        goalDist = odometry.getDistanceFromGoal();
        targetTPS = odometry.newDynamicTargetTPS(goalDist);


        // Subsystems
        intake.runIntake(gamepad1);
//        parking.update(gamepad1);
        flywheel.cycleShootingState(gamepad1, gamepad2);
        flywheel.setTargetTPS(targetTPS);
        flywheel.update(gamepad1);

        updateTelemetry(telemetry);
    }

    private void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Mode", "MANUAL (Field Centric)");
        telemetry.addData("Shooter State", flywheel.getShotState());
        telemetry.addData("Flywheel Velocity", flywheel.getVelocity());
        telemetry.addData("Target Heading", Math.toDegrees(odometry.getGoalAngle()));
        telemetry.addData("Current Heading", Math.toDegrees(odometry.getHeading()));
        telemetry.addData("Error Degrees", Math.toDegrees(odometry.getError()));
        telemetry.addData("Drive X", odometry.getRobotX());
        telemetry.addData("Drive Y", odometry.getRobotY());
        telemetry.addData("Hood Pos", flywheel.getHoodPos());
        telemetry.addData("Target Velocity", flywheel.getTargetTPS());
        telemetry.addData("Goal Dist", goalDist);

//        telemetry.addData("Park Position", parking.getParkPosition());
//        telemetry.addData("Drive Position", parking.getDrivePosition());
        telemetry.update();
    }
}