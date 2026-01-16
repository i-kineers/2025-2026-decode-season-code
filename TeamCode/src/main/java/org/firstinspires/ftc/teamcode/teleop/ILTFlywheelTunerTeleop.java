package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ILTFlywheelTunerSubsystem;

@TeleOp(name = "ILTFlywheelTunerTeleop", group = "Tuning")
public class ILTFlywheelTunerTeleop extends OpMode {

    private ILTFlywheelTunerSubsystem flywheelTuner;

    @Override
    public void init() {
        flywheelTuner = new ILTFlywheelTunerSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        flywheelTuner.updateShooter(flywheelTuner.targetVelocity);
        flywheelTuner.handleGamepadInputs(gamepad1);
        flywheelTuner.updateTelemetry(telemetry);
    }
}
