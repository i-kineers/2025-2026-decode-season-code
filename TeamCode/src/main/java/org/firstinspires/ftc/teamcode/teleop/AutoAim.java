package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.TeleOpPathingManager;

@TeleOp(name = "Auto Aim TeleOp", group = "TeleOp")
public class AutoAim extends OpMode {

    private TeleOpPathingManager pathingManager;
    private Camera camera;
    private FieldCentricDrive chassis; 
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    @Override
    public void init() {
        pathingManager = new TeleOpPathingManager(hardwareMap, new Pose(22, 120, Math.toRadians(135)));
        camera = new Camera(hardwareMap);
        chassis = new FieldCentricDrive(hardwareMap);
    }

    @Override
    public void start() {
        // Pathing manager starts automatically
        camera.resumeStream();
    }

    @Override
    public void loop() {
        pathingManager.update();

        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        boolean isHoldingAlign = gamepad1.x;

        // Auto Aim Logic Override
        if (isHoldingAlign && !pathingManager.isAutomated()) {
            Double turnPower = camera.useBearingToAlign();
            if (turnPower != null) {
                // Override turn with PID output
                turn = turnPower; 
                // Optionally zero out translation if you want to stop while aiming
                // forward = 0; 
                // strafe = 0;
            }
        }

        // Pass control to manager
        pathingManager.drive(forward, strafe, turn, gamepad1.a);

        // IMU Reset
        if (gamepad1.b) {
             if (chassis != null) {
                 chassis.resetIMU();
                 // Also reset follower heading if needed, though chassis reset might be enough depending on your setup
                 Pose current = pathingManager.getFollower().getPose();
                 pathingManager.getFollower().setPose(new Pose(current.getX(), current.getY(), 0));
             }
        }

        // Telemetry
        Pose currentPose = pathingManager.getFollower().getPose();
        if (currentPose != null) {
            panelsTelemetry.getTelemetry().addData("Robot X", currentPose.getX());
            panelsTelemetry.getTelemetry().addData("Robot Y", currentPose.getY());
            panelsTelemetry.getTelemetry().addData("Robot H", Math.toDegrees(currentPose.getHeading()));
            panelsTelemetry.getTelemetry().update();

            telemetry.addData("Mode", pathingManager.isAutomated() ? "Pathing" : (isHoldingAlign ? "Auto Aim" : "Manual"));
            if (isHoldingAlign) {
                telemetry.addData("Camera Error", camera.returnError());
            }
        }
        telemetry.update();
    }
    
    @Override
    public void stop() {
        camera.stopStream();
    }
}
