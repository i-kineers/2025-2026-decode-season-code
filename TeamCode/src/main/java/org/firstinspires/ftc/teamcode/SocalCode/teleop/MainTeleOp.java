package org.firstinspires.ftc.teamcode.SocalCode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.MasterLogic;

@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends OpMode {

    private MasterLogic master;
    private boolean allianceBlue = true; // Default to Blue
    private boolean previousYState = false; // For debouncing

    @Override
    public void init() {
        telemetry.addLine("Main TeleOp Initialized.");
        telemetry.addLine("Wait for side selection in init_loop...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // --- Debounce Logic for Toggle ---
        // Checks if Y is pressed now, but wasn't pressed in the last loop
        if (gamepad1.y && !previousYState) {
            allianceBlue = !allianceBlue; // Toggle the boolean
        }
        previousYState = gamepad1.y; // Update the previous state

        // --- Telemetry Display ---
        telemetry.addLine("=== PRE-MATCH SELECTION ===");
        telemetry.addData("Selected Alliance", allianceBlue ? "BLUE" : "RED");
        telemetry.addLine("Press (Y) on Gamepad 1 to Toggle");
        telemetry.addLine("----------------------------------");
        telemetry.addLine("Press START when ready.");
        telemetry.update();
    }

    @Override
    public void start() {
        // Now master is initialized with the final selection from init_loop
        if (allianceBlue) {
            master = new MasterLogic(hardwareMap, 22, 120, 135, allianceBlue);
        } else if (!allianceBlue) {
            master = new MasterLogic(hardwareMap, 122, 120, 45, allianceBlue);
        }
    }

    @Override
    public void loop() {
        master.mainLogic(gamepad1, gamepad2, telemetry);
    }
}