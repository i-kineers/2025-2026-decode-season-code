package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TESTINGGG")
public class RamsRPMcalculator extends OpMode {

    private BasicOuttake outtake;
    private double currentPower = 0.0;  // current motor power (0.0 to 1.0)
    private final double CHANGE_RATE = 0.01; // how fast power changes per loop
    static final double TICK_PER_MOTOR_REV = 537.7;    // eg: GoBuilda Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
//    static final double TICKS_PER_INCH = (TICK_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    ElapsedTime timer;

    @Override
    public void init() {
        outtake = new BasicOuttake(hardwareMap);
        int lastPosition = outtake.getLauncherPosition();
        timer = new ElapsedTime();
    }

    int lastPosition;
    double RPM = 0;
    @Override
    public void loop() {
        if (timer.seconds() >= 0.1) {
            int currentPosition = outtake.getLauncherPosition();
            int deltaTicks = currentPosition - lastPosition;
            double deltaTime = timer.seconds(); // time in seconds

            RPM = calculateRPM(deltaTicks, TICK_PER_MOTOR_REV, deltaTime);
        }

        telemetry.update();
    }
    private double calculateRPM(int deltaTicks, double ticksPerRev, double deltaTimeSec) {
        if (ticksPerRev <= 0 || deltaTimeSec <= 0) return 0;
        double revolutions = (double) deltaTicks / ticksPerRev;
        return revolutions * (60.0 / deltaTimeSec);
    }
}

