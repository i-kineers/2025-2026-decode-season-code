package org.firstinspires.ftc.teamcode.CodePriorILT.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.CodePriorILT.subsystems.MasterLogic;

@TeleOp(name = "Red Far TeleOp", group = "Main")
public class redFarTeleop extends OpMode {

    private MasterLogic master;

    @Override
    public void init() {
        master = new MasterLogic(hardwareMap, 107.961, 14.079, 90, false);

        telemetry.addLine("Red Far TeleOp Initialized.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Delegate all robot behavior to the MasterLogic class
        master.mainLogic(gamepad1, gamepad2, telemetry);
    }
}
