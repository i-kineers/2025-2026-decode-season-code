package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.dfrobot.HuskyLens.Block;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
@TeleOp(name = "HuskyLens", group = "Sensor")
public class HuskylensOpmodeTest extends OpMode {
    private final int READ_PERIOD = 1;


    private HuskyLens huskyLens;
    Deadline rateLimit;

    @Override
    public void init() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }


        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.update();
    }

    public void loop() {
        if (!rateLimit.hasExpired()) {
            return;
        }
        rateLimit.reset();

        Block[] blocks = huskyLens.blocks();
        telemetry.addData("Detected Tags", blocks.length);
        for (Block block : blocks) {
            // Basic tag data
            telemetry.addData("Block", block.toString());
            telemetry.addData("Tag ID", block.id);
            telemetry.addData("Position", "X: %d  Y: %d", block.x, block.y);
            telemetry.addData("Size", "Width: %d  Height: %d", block.width, block.height);

            // Example: Decide location based on X position (like AprilTag zones)
            if (block.x < 150) {
                telemetry.addLine(" Tag on LEFT");
            } else if (block.x > 170) {
                telemetry.addLine(" Tag on RIGHT");
            } else {
                telemetry.addLine(" Tag in CENTER");
            }
            if (block.y < 110) {
                telemetry.addLine(" Tag on DOWNWARDS");
            } else if (block.y > 130) {
                telemetry.addLine(" Tag on UPWARDS");
            } else {
                telemetry.addLine(" Tag in CENTER");
            }
            if (block.y < 130 && block.y >110 && block.x < 170 && block.x > 150){
                telemetry.addLine(" Tag in PERFECT CENTER");
            }
            // Example: Estimate distance using tag size
            if (block.height > 80) {
                telemetry.addLine(" Tag is CLOSE");
            } else {
                telemetry.addLine(" Tag is FAR");
            }
            if (block.width > 80) {
                telemetry.addLine(" Tag is CLOSE");
            } else {
                telemetry.addLine(" Tag is FAR");
            }

        }
        telemetry.update();
    }
}

