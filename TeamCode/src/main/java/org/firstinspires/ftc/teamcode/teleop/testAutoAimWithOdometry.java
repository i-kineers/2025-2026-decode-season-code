package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.AutoAimWithOdometry;

@TeleOp(name="Heading Lock")
public class testAutoAimWithOdometry extends OpMode {

    AutoAimWithOdometry autoAimWithOdometry;

    @Override
    public void init() {
        // FIXED: Added 'new' keyword to initialize the object
        autoAimWithOdometry = new AutoAimWithOdometry(hardwareMap);
    }

    @Override
    public void loop() {
        // Toggle the lock state based on the trigger
        if (gamepad1.aWasPressed()) {
            autoAimWithOdometry.setHeadingLock(true);
        } else if (gamepad1.bWasPressed()){
            autoAimWithOdometry.setHeadingLock(false);
        }

        // Pass inputs to the subsystem
        autoAimWithOdometry.update(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        // Optional: Add telemetry to see if it's working
        telemetry.addData("Heading Lock", autoAimWithOdometry.isHeadingLockEnabled());
        telemetry.addData("Target Heading (deg)", autoAimWithOdometry.getTargetHeadingDeg());
        telemetry.addData("Current Heading (deg)", autoAimWithOdometry.getCurrentHeadingDeg());
        telemetry.addData("Heading Error (deg)", autoAimWithOdometry.getHeadingErrorDeg());
        telemetry.addData("Turn Output", autoAimWithOdometry.getTurnPower());
        telemetry.update();

    }
}
