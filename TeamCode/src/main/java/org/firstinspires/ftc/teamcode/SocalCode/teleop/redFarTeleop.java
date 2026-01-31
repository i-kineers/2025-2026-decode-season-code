package org.firstinspires.ftc.teamcode.SocalCode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.MasterLogic;

@TeleOp(name = "Red Far TeleOp", group = "Main")
public class redFarTeleop extends OpMode {

    private MasterLogic master;

    @Override
    public void init() {
        // Initialize the logic master
        // Starting pose from testFarRed: (91.031, 11.2525, 65 degrees)
        master = new MasterLogic(hardwareMap, 110.546, 11.151, 90, false);

        telemetry.addLine("Red Far TeleOp Initialized.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Delegate all robot behavior to the MasterLogic class
        master.mainLogic(gamepad1, gamepad2, telemetry);
    }
}
