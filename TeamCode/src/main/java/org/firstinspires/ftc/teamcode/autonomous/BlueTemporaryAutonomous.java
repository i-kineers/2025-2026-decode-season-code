package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.subsystems.DoubleMotorOuttakePID;

@Autonomous(name = "Blue Temporary Autonomous", group = "Temporary")
public class BlueTemporaryAutonomous extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor rearLeft;
    private DcMotor frontRight;
    private DcMotor rearRight;
    private DoubleMotorOuttakePID outtake;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "flmotor");
        rearLeft = hardwareMap.get(DcMotor.class, "blmotor");
        frontRight = hardwareMap.get(DcMotor.class, "frmotor");
        rearRight = hardwareMap.get(DcMotor.class, "brmotor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake = new DoubleMotorOuttakePID(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            // Move backwards for 2 seconds
            moveBackwards(0.5, 800);

            // Shoot
            outtake.autoRapidShoot(2500, 5000); // Shoot at 2500 RPM for 1 second

            // Strafe right a little more to cross the line
            strafeRight(0.5, 500);
        }
    }

    private void moveForwards(double power, long time) {
        frontLeft.setPower(power);
        rearLeft.setPower(power);
        frontRight.setPower(power);
        rearRight.setPower(power);
        sleep(time);
        stopRobot();
    }

    private void moveBackwards(double power, long time) {
        frontLeft.setPower(-power);
        rearLeft.setPower(-power);
        frontRight.setPower(-power);
        rearRight.setPower(-power);
        sleep(time);
        stopRobot();
    }

    private void strafeLeft(double power, long time) {
        frontLeft.setPower(-power);
        rearLeft.setPower(power);
        frontRight.setPower(power);
        rearRight.setPower(-power);
        sleep(time);
        stopRobot();
    }

    private void strafeRight(double power, long time) {
        frontLeft.setPower(power);
        rearLeft.setPower(-power);
        frontRight.setPower(-power);
        rearRight.setPower(power);
        sleep(time);
        stopRobot();
    }

    private void stopRobot() {
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }
}
