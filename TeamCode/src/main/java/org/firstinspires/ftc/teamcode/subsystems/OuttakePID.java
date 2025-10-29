package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OuttakePID {

    // Set PID constants
    public static int LAUNCHER_MIN_POS = 0;       // The fully retracted position
    public static int LAUNCHER_MAX_POS = 2500;    // The fully extended position (EXAMPLE VALUE, you must find the real one)
    private int targetSlidePosition = 0;
    private static double kP = 0.1;
    private static double kI = 0.001;
    private static double kD = 0.00;
    private double p_component = 0.0;
    private double i_component = 0.0;
    private double d_component = 0.0;
    private double raw_pid_value = 0.0;
    private double lastCalculatedPower = 0.0;
    private double previous_time;
    private double previous_error;
    private final ElapsedTime runtime;
    private final ElapsedTime switchTime;

    // Set RPM calculator constants
    private BasicOuttake outtake;
    private double currentPower = 0.0;  // current motor power (0.0 to 1.0)
    private final double CHANGE_RATE = 0.01; // how fast power changes per loop
    static final double TICK_PER_MOTOR_REV = 537.7;    // eg: GoBuilda Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private final ElapsedTime timer;
    private int lastPosition = 0;
    private double lastCalculatedRPM = 0.0;

    // Declare motors and servos
    private DcMotor launcher;
    private CRServo sideLauncher1;
    private CRServo sideLauncher2;

    public OuttakePID(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotor.class, "launcher"); // Initialize the member variable
        sideLauncher1 = hardwareMap.get(CRServo.class, "leftLoader"); // Initialize the member variable
        sideLauncher2 = hardwareMap.get(CRServo.class, "rightLoader"); // Initialize the member variable
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runtime = new ElapsedTime();
        switchTime = new ElapsedTime();
        timer = new ElapsedTime();
        this.lastPosition = launcher.getCurrentPosition(); // FIX #2:
    }

    private double runLauncher() {
        // MODIFICATION #1: Added safety check to prevent crash on first loop
        if (previous_time == 0) {
            previous_time = runtime.milliseconds();
        }

        double current_time = runtime.milliseconds();
        double time_delta = current_time - previous_time;

        // MODIFICATION #2: Added safety check to prevent division by zero
        if (time_delta == 0) {
            return lastCalculatedPower;
        }

        double current_error = targetSlidePosition - launcher.getCurrentPosition();

        double p = kP * current_error;
        double i = kI * (current_error * time_delta);
        double d = kD * (current_error - previous_error) / time_delta;

        this.p_component = kP * current_error;
        this.i_component += kI * (current_error * time_delta); // Integral is cumulative
        this.d_component = kD * (current_error - previous_error) / time_delta;
        this.raw_pid_value = p_component + i_component + d_component;

        previous_time = current_time;
        previous_error = current_error;

        double PID = p + i + d;
        // Control intake slides motor
        return Math.max(-1.0, Math.min(PID, 1.0));
    }

    private void RPMCalculator() {
        int currentPosition = launcher.getCurrentPosition();
        double currentTime = timer.milliseconds();
        double deltaTime = currentTime;

        if (deltaTime >= 50) { // 50ms
            int deltaTicks = currentPosition - lastPosition;
            this.lastCalculatedRPM = RPMFormula(deltaTicks, TICK_PER_MOTOR_REV, deltaTime / 1000.0);

            this.lastPosition = currentPosition;
            timer.reset(); // reset AFTER calculation
        }
    }


    private double RPMFormula(int deltaTicks, double ticksPerRev, double deltaTimeSec) {
        if (ticksPerRev <= 0 || deltaTimeSec <= 0) return 0;
        double revolutions = (double) deltaTicks / ticksPerRev;
        return revolutions * (60.0 / deltaTimeSec);
    }

    public void update() {
        if (this.targetSlidePosition == 0) {
            launcher.setPower(0);
        } else {
            double power = runLauncher();
            this.lastCalculatedPower = power;
            launcher.setPower(power);
            RPMCalculator();
        }
    }

    public void decreaseMaxPosition() {
        if (LAUNCHER_MAX_POS <= 10) {
            LAUNCHER_MAX_POS = 10;
        } else {
            LAUNCHER_MAX_POS -= 10;
        }
    }

    public void increaseMaxPosition() {
        LAUNCHER_MAX_POS += 10; // Increase by 50 ticks
    }

    public void setTargetPosition(int ticks) {
        // Clamp the input 'ticks' to be between LAUNCHER_MIN_POS and LAUNCHER_MAX_POS
        this.targetSlidePosition = Math.max(LAUNCHER_MIN_POS, Math.min(LAUNCHER_MAX_POS, ticks));
    }
    public void setMotorPowerPID(double power) {
        launcher.setPower(runLauncher());
    }

    public int getTargetPosition() {
        return targetSlidePosition;
    }

    public int getCurrentPosition() {
        return launcher.getCurrentPosition();
    }

    public double getCalculatedPower() {
        return this.lastCalculatedPower;
    }

    // MODIFICATION #4: Added a necessary getter for the RPM value
    public double getRPM() {
        return this.lastCalculatedRPM;
    }

//    public double getP() {
//        return this.p_component;
//    }
//
//    public double getI() {
//        return this.i_component;
//    }
//
//    public double getD() {
//        return this.d_component;
//    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getPID() {
        return this.raw_pid_value;
    }

    public void increaseI() {
        if (kI >= 0.001) {
            // If kI is relatively large, increase it by a larger amount
            kI += 0.001;
        } else {
            // Otherwise, use a very small increment for fine-tuning
            kI += 0.0001;
        }
    }

    public void decreaseI() {
        if (kI > 0.001) {
            // If kI is relatively large, decrease it by a larger amount
            kI -= 0.001;
        } else {
            // Otherwise, use a very small decrement for fine-tuning
            kI -= 0.0001;
        }

        // Safety check to ensure kI never goes negative
        if (kI < 0) {
            kI = 0;
        }
    }

    public double increaseP() {
        if (kP > 0.01) {
            kP += 0.01;
        } else if (kP >= 0 && kP <= 0.01) {
            kP += 0.001;
        }
        return kP;
    }

    public double decreaseP() {
        if (kP > 0.01) {
            kP -= 0.01;
        } else if (kP >= 0 && kP <= 0.01) {
            kP -= 0.001;
        }

        return kP;
    }

    public void stop() {
        // 1. Immediately stop the motor's physical movement.
        launcher.setPower(0.0);

        // 2. Reset the internal state of the PID controller.
        // This is what makes the graph lines go to zero and prevents integral windup.
        this.i_component = 0.0;     // <-- THIS IS THE MOST CRITICAL LINE.
        this.p_component = 0.0;
        this.d_component = 0.0;
        this.raw_pid_value = 0.0;
        this.lastCalculatedPower = 0.0;
        this.previous_error = 0.0;

        // 3. Reset the target so the graph is accurate when stopped.
        this.targetSlidePosition = 0;

        // 4. Reset timer state to be ready for the next run.
        runtime.reset();
        previous_time = 0;

        // 5. Optional but good practice: Reset the encoder reference point.
        // This ensures the next "run" command starts counting from a true zero.
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
