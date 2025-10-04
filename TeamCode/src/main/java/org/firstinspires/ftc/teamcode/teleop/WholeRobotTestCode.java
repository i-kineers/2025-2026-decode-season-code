package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MessyTeleOp")
public class WholeRobotTestCode extends LinearOpMode {

    private Drive drive;
    private Arm arm;
    private Claw claw;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drive(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            // Drive
            drive.rundrive(gamepad1);
            arm.runarm(gamepad2);
            claw.runclaw(gamepad2);


            // Telemetry
            telemetry.addData("LF Power", drive.getlfpower());
            telemetry.addData("RF Power", drive.getrfpower());
            telemetry.addData("Arm Power", arm.getArmPower());
            telemetry.addData("Claw Pos", claw.getClawPos());
            telemetry.update();
        }
    }
}