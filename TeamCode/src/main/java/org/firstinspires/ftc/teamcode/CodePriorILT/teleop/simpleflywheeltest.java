package org.firstinspires.ftc.teamcode.CodePriorILT.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * FLYWHEEL kF CALIBRATION TOOL
 * * Instructions:
 * 1. Support the robot so the flywheel can spin safely.
 * 2. Use a charged battery (12.5V+).
 * 3. Run the OpMode and press START.
 * 4. The robot will spin at two different voltages and calculate the normalized kF.
 * 5. Record the results from the screen/Dashboard.
 */

@TeleOp(name="simpleflywheeltest", group="Calibration")
public class simpleflywheeltest extends LinearOpMode {

    private DcMotorEx flywheel2;
    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() {
        // Initialize Hardware
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Ensure motor is in the right mode for raw testing
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {
            // Get gamepad input for motor power
            double motorPower = -gamepad1.left_stick_y; // Use left stick for power

            // Set motor power
            flywheel2.setPower(motorPower);

            // Display current velocity (tps)
            double currentVelocity = flywheel2.getVelocity();
            telemetry.addData("Flywheel Velocity (TPS)", currentVelocity);
            telemetry.addData("Motor Power", motorPower);
            telemetry.update();
        }
    }
}