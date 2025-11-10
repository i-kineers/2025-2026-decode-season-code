package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DoubleMotorOuttakePID;

@TeleOp(name = "Test to find rpm for given decimal", group = "Tests")
public class FShooterTest extends OpMode {
    private DoubleMotorOuttakePID shooter;
    double motorPower = 0.0;

    @Override
    public void init() {
        shooter = new DoubleMotorOuttakePID(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.aWasPressed()) {
            motorPower += 0.01;
        } else if (gamepad1.bWasPressed()) {
            motorPower -= 0.01;
        } else if (gamepad1.xWasPressed()) {
            shooter.setPower(motorPower);
        }

        // 4. Provide feedback to the screen
        telemetry.addData("Motor Power", motorPower);
        telemetry.addData("RPM", shooter.getRawRPM());
        telemetry.addData(">", "B: Decrease Power");
        telemetry.addData(">", "A: Increase Power");
        telemetry.update();

        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
