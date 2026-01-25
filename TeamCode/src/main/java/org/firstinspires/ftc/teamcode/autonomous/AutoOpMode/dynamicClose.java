package org.firstinspires.ftc.teamcode.autonomous.AutoOpMode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.AutonomousCycleManager;

@Autonomous(name = "Close Auto", group = "Main")
public class dynamicClose extends OpMode {

    private AutonomousCycleManager autoManager;

    // UI variables for path selection
    private boolean[] toggles = {false, false, false, false};
    private int cursor = 0;
    private boolean isBlueAlliance = true; // Default to Blue

    // Debounce flags
    private boolean upPrev = false;
    private boolean downPrev = false;
    private boolean aPrev = false;
    private boolean yPrev = false;

    @Override
    public void init() {
        // Manager will be initialized in init_loop after selection
        telemetry.addLine("Ready for path selection...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // --- Path Selection UI ---
        telemetry.addLine("=== SELECT PATHS ===");
        telemetry.addLine("Use D-Pad Up/Down to move");
        telemetry.addLine("Press A to toggle Intake");
        telemetry.addLine("Press Y to toggle Alliance\n");

        // Alliance Color Selection
        String allianceColor = isBlueAlliance ? "BLUE" : "RED";
        telemetry.addData("Alliance (Y)", allianceColor);
        telemetry.addLine();

        // Intake Path Selection
        for (int i = 0; i < 4; i++) { // Changed 3 to 4
            String arrow = (i == cursor) ? ">" : " ";
            String state = toggles[i] ? "ON" : "off";
            String label = (i < 3) ? " Intake " + (i + 1) : " Open Gate"; // Added gate label
            telemetry.addData(arrow + label, state); // Changed "Intake" to label
        }
        telemetry.update();

        // --- Handle UI input ---

        // Toggle Alliance Color
        if (gamepad1.y && !yPrev) {
            isBlueAlliance = !isBlueAlliance;
        }

        // Move cursor up
        if (gamepad1.dpad_up && !upPrev) {
            cursor = (cursor - 1 + 4) % 4; // Changed 3 to 4
        }
        // Move cursor down
        if (gamepad1.dpad_down && !downPrev) {
            cursor = (cursor + 1) % 4; // Changed 3 to 4
        }
        // Toggle current intake path
        if (gamepad1.a && !aPrev) {
            toggles[cursor] = !toggles[cursor];
        }

        // Update debounce flags
        upPrev = gamepad1.dpad_up;
        downPrev = gamepad1.dpad_down;
        aPrev = gamepad1.a;
        yPrev = gamepad1.y;
    }

    @Override
    public void start() {
        autoManager = new AutonomousCycleManager(hardwareMap, isBlueAlliance);
        autoManager.setCycles(toggles[0], toggles[1], toggles[2], toggles[3]);
    }

    @Override
    public void loop() {
        if (autoManager != null) {
            autoManager.update();

            // --- Telemetry ---
            Pose currentPose = autoManager.getFollower().getPose();
            telemetry.addData("State", autoManager.getCurrentState());
            if (currentPose != null) {
                telemetry.addData("X", currentPose.getX());
                telemetry.addData("Y", currentPose.getY());
                telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
            }
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        // OpMode will stop automatically
    }
}