package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="MessyTeleOp")
public class UnmessyTeleOp extends LinearOpMode {
    private DriveWithoutGP drive;
    private ArmWithoutGP arm;
    private ClawWithoutGP claw;

    double clawPos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        drive= new DriveWithoutGP(hardwareMap);
         arm = new ArmWithoutGP(hardwareMap);
        claw = new ClawWithoutGP(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            // Drive
            drive.rundriveX(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            if (gamepad2.dpad_up) {
               arm.moveUp();
            } else if (gamepad2.dpad_down) {
                arm.moveDown();
            } else {
                arm.Stop();
            }

            if (gamepad2.a) {
                 claw.a();
                 
            } else if (gamepad2.b) {
                claw.b();
               
            }
            

            // Telemetry
            telemetry.addData("LF Power", drive.getlfpower());
            telemetry.addData("RF Power", drive.getrfpower());
            telemetry.addData("Arm Power", arm.getArmPower());
            telemetry.addData("Claw Pos", clawPos);
            telemetry.update();
        }
    }
}

