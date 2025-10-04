package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name="PID Test", group="PID")
    public class pid extends LinearOpMode {


        // Define your motors
        DcMotor leftDrive;
        DcMotor rightDrive;


        // --- STUDENT: CALCULATE AND SET THESE VALUES ---
        // Ticks per revolution for your motor
        static final double TICKS_PER_REV = 537.7; // Example for a GoBILDA 5203 series motor
        // The gear ratio for your drivetrain
        static final double GEAR_RATIO = 1.0; // This is for a 1:1 direct drive. Change this if you have gears.
        // Diameter of your wheels in inches
        static final double WHEEL_DIAMETER_INCHES = 4.0;
        // Calculate ticks per inch
        static final double TICKS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * 3.14159);


        // The distance you want to travel in inches
        static final double DRIVE_DISTANCE = 24.0;


        @Override
        public void runOpMode() {
            // Map the motors from the robot's configuration
            leftDrive = hardwareMap.get(DcMotor.class, "left_drive"); // <-- Change to your motor's name
            rightDrive = hardwareMap.get(DcMotor.class, "right_drive"); // <-- Change to your motor's name


            // IMPORTANT: Reverse the right motor so it spins the correct way.
            // This may need to be DcMotor.Direction.FORWARD if your robot drives backwards.
            leftDrive.setDirection(DcMotor.Direction.REVERSE);


            telemetry.addData("Status", "Ready to run");
            telemetry.update();


            waitForStart();


            // Step 1: Calculate the target ticks
            int targetTicks = (int)(DRIVE_DISTANCE * TICKS_PER_INCH);


            // Step 2: Reset the encoders and set the target positions
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            leftDrive.setTargetPosition(targetTicks);
            rightDrive.setTargetPosition(targetTicks);


            // Step 3: Set the motors to RUN_TO_POSITION mode
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // Step 4: Set a power level for the motors
            leftDrive.setPower(1);
            rightDrive.setPower(1);


            // Step 5: Loop until both motors have reached their target
            while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
                // Display telemetry
                telemetry.addData("Status", "Running to position...");
                telemetry.addData("Left Position", "%d / %d", leftDrive.getCurrentPosition(), targetTicks);
                telemetry.addData("Right Position", "%d / %d", rightDrive.getCurrentPosition(), targetTicks);
                telemetry.update();
            }


            // Step 6: Stop the motors
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftDrive.setPower(0);
            rightDrive.setPower(0);


            // Optional: Reset the mode for TeleOp
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            telemetry.addData("Status", "Movement Complete");
            telemetry.update();


            // Keep the OpMode alive to view telemetry
            sleep(5000);
        }
    }




