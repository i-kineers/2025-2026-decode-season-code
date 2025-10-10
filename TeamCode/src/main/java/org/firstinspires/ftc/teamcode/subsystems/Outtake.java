package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {

    // PID gains
    private static final double kP = 0.004;
    private static final double kI = 0.000;
    private static final double kD = 0.000;

    private double targetRPM = 1000;   // desired flywheel speed
    private double integral = 0;
    private double previousError = 0;
    private double previousTime = 0;
    private int previousPosition = 0;

    private ElapsedTime runtime = new ElapsedTime();

    private CRServo leftLoader;
    private CRServo rightLoader;
    private DcMotor launcher;

    public Outtake(HardwareMap hardwareMap) {
        leftLoader = hardwareMap.get(CRServo.class, "leftLoader");
        rightLoader = hardwareMap.get(CRServo.class, "rightLoader");
        launcher = hardwareMap.get(DcMotor.class, "launcher");

        // Reset and prepare encoder
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        previousTime = runtime.seconds();
        previousPosition = launcher.getCurrentPosition();
    }

    /**
     * Runs the PID loop for flywheel velocity (RPM).
     */
    public void runLauncherPID() {
        double currentTime = runtime.seconds();
        double dt = currentTime - previousTime;
        if (dt <= 0) dt = 1e-3; // prevent division by zero

        int currentPosition = launcher.getCurrentPosition();
        double ticksPerSec = (currentPosition - previousPosition) / dt;

        // motor encoder CPR (counts per revolution)
        double ticksPerRev = 28.0;  // GoBILDA Yellow Jacket motor default
        double currentRPM = (ticksPerSec / ticksPerRev) * 60.0;

        // PID error on velocity
        double error = targetRPM - currentRPM;
        integral += error * dt;
        double derivative = (error - previousError) / dt;

        double output = kP * error + kI * integral + kD * derivative;

        // Clamp power so it doesn't exceed [-1, 1]
        output = Math.max(-1.0, Math.min(1.0, output));

        launcher.setPower(output);

        // Save for next loop
        previousError = error;
        previousTime = currentTime;
        previousPosition = currentPosition;
    }

    /**
     * Sets a new target RPM for the launcher flywheel.
     */
    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    /**
     * Example stub for outtake control (load servos).
     */
    public void runOuttake() {
        leftLoader.setPower(1.0);
        rightLoader.setPower(-1.0);
    }
}
