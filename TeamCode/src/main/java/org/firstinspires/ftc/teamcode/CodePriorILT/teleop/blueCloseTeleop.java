package org.firstinspires.ftc.teamcode.CodePriorILT.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.CodePriorILT.subsystems.MasterLogic;

@TeleOp(name = "Blue Close TeleOp", group = "Main")
public class blueCloseTeleop extends OpMode {

    private MasterLogic master;

    @Override
    public void init() {
        // Initialize the logic master
        master = new MasterLogic(hardwareMap, 58, 112, 180, true);

        telemetry.addLine("Blue Close TeleOp Initialized.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Delegate all robot behavior to the MasterLogic class
        master.mainLogic(gamepad1, gamepad2, telemetry);
    }
}
