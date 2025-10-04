package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


    @TeleOp(name="MessyTeleOp")
    public class MessyTeleOp extends LinearOpMode {
        DcMotor leftFront, rightFront, leftBack, rightBack;
        DcMotor armMotor;
        Servo clawServo;

        double clawPos = 0.5;

        @Override
        public void runOpMode() throws InterruptedException {
            leftFront = hardwareMap.get(DcMotor.class, "lf");
            rightFront = hardwareMap.get(DcMotor.class, "rf");
            leftBack = hardwareMap.get(DcMotor.class, "lb");
            rightBack = hardwareMap.get(DcMotor.class, "rb");
            armMotor = hardwareMap.get(DcMotor.class, "armMotor");
            clawServo = hardwareMap.get(Servo.class, "clawServo");

            waitForStart();

            while (opModeIsActive()) {
                // Drive
                double drive = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                double lfPower = drive + strafe + turn;
                double rfPower = drive - strafe - turn;
                double lbPower = drive - strafe + turn;
                double rbPower = drive + strafe - turn;

                leftFront.setPower(lfPower);
                rightFront.setPower(rfPower);
                leftBack.setPower(lbPower);
                rightBack.setPower(rbPower);

                // Arm
                if (gamepad2.dpad_up) {
                    armMotor.setPower(0.5);
                } else if (gamepad2.dpad_down) {
                    armMotor.setPower(-0.5);
                } else {
                    armMotor.setPower(0);
                }

                // Claw
                if (gamepad2.a) {
                    clawPos += 0.01;
                } else if (gamepad2.b) {
                    clawPos -= 0.01;
                }
                clawPos = Math.max(0, Math.min(1, clawPos));
                clawServo.setPosition(clawPos);

                // Telemetry
                telemetry.addData("LF Power", lfPower);
                telemetry.addData("RF Power", rfPower);
                telemetry.addData("Arm Power", armMotor.getPower());
                telemetry.addData("Claw Pos", clawPos);
                telemetry.update();
            }
        }
    }

