//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.subsystems.MasterLogic;
//
//@TeleOp(name = "Main TeleOp", group = "Main")
//public class MainTeleOp extends OpMode {
//
//    private MasterLogic master;
//
//    @Override
//    public void init() {
//        // Initialize the logic master
//        master = new MasterLogic(hardwareMap);
//
//        telemetry.addLine("Main TeleOp Initialized.");
//        telemetry.update();
//    }
//
//    @Override
//    public void loop() {
//        // Delegate all robot behavior to the MasterLogic class
//        master.mainLogic(gamepad1, gamepad2, telemetry);
//    }
//}
