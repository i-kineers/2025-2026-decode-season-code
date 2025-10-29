package org.firstinspires.ftc.teamcode.teleop;

// This is the correct import for the syntax you are using.
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.OuttakePID;

@TeleOp(name = "Outtake PID Test")
public class OuttakePIDTest extends LinearOpMode {
    private OuttakePID outtakePID;

    // 1. Correctly declare and initialize the PanelsTelemetry instance using .INSTANCE
    private PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the subsystem
        outtakePID = new OuttakePID(hardwareMap);

        // 2. REMOVED the incorrect initialization: 'new PANELS(this).getTelemetry()'

        // You can still use the standard telemetry for initial instructions on the Driver Hub
        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData(">", "Connect to the Panels dashboard now.");
        telemetry.addData(">", "Press A to go to MAX position");
        telemetry.addData(">", "Press B to go to MIN position");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Control Logic ---
            if (gamepad1.a) {
                outtakePID.setTargetPosition(OuttakePID.LAUNCHER_MAX_POS);
            } else if (gamepad1.b) {
//                outtakePID.setTargetPosition(OuttakePID.LAUNCHER_MIN_POS);
                outtakePID.stop();
            } else if (gamepad1.xWasPressed()) {
                outtakePID.increaseMaxPosition();
            } else if (gamepad1.yWasPressed()) {
                outtakePID.decreaseMaxPosition();
            } else if (gamepad1.leftBumperWasPressed()) {
                outtakePID.increaseP();
            } else if (gamepad1.rightBumperWasPressed()) {
                outtakePID.decreaseP();
            } else if (gamepad1.dpadRightWasPressed()) {
                outtakePID.increaseI();
            } else if (gamepad1.dpadLeftWasPressed()) {
                outtakePID.decreaseI();
            }


            // This runs the PID calculations and sets the motor power.
            outtakePID.update();


            // 3. Use the correct syntax for sending data, as you did in TestGraph
            panelsTelemetry.getTelemetry().addData("Target Position", outtakePID.getTargetPosition());
            panelsTelemetry.getTelemetry().addData("Current Position", outtakePID.getCurrentPosition());
            panelsTelemetry.getTelemetry().addData("Calculated Power", outtakePID.getCalculatedPower());
            panelsTelemetry.getTelemetry().addData("RPS", outtakePID.getRPM());

            panelsTelemetry.getTelemetry().addData("P_Component", outtakePID.getP());
            panelsTelemetry.getTelemetry().addData("I_Component", outtakePID.getI());
//            panelsTelemetry.getTelemetry().addData("D_Component", outtakePID.getD());
            panelsTelemetry.getTelemetry().addData("Raw PID Value", outtakePID.getPID());


            panelsTelemetry.getTelemetry().update(telemetry);

            // A small sleep helps keep the loop consistent and reduces CPU load
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
