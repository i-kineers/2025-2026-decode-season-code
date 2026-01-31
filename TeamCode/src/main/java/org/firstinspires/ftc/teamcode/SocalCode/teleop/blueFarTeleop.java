package org.firstinspires.ftc.teamcode.SocalCode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.MasterLogic;

@TeleOp(name = "Blue Far TeleOp", group = "Main")
public class blueFarTeleop extends OpMode {

    private MasterLogic master;

    @Override
    public void init() {
        // Initialize the logic master
        // Starting pose from testFarBlue: (52.969, 11.151, 110 degrees)
        master = new MasterLogic(hardwareMap, 33.454,11.151, 90, true);

        telemetry.addLine("Blue Far TeleOp Initialized.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Delegate all robot behavior to the MasterLogic class
        master.mainLogic(gamepad1, gamepad2, telemetry);
    }
}
