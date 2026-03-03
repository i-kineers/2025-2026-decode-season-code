package org.firstinspires.ftc.teamcode.CodePriorILT.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.CodePriorILT.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.CodePriorILT.subsystems.AutoAimWithOdometry;
import org.firstinspires.ftc.teamcode.CodePriorILT.subsystems.FlywheelSystem;


import static org.firstinspires.ftc.teamcode.CodePriorILT.pedroPathing.Tuning.follower;

public class flywheelwithodomtestingtofinddistance extends OpMode {
    private AutoAimWithOdometry autoAimWithOdometry;
    private FlywheelSystem flywheel;
    private double targetTPS = 1200;
    private double idleTPS = 0;

    private Pose startingPose;
    @Override
    public void init (){
        autoAimWithOdometry = new AutoAimWithOdometry(hardwareMap, true);
        flywheel = new FlywheelSystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
    }
    @Override
    public void loop(){
        flywheel.handleTriggerInput(gamepad1.right_trigger, targetTPS, idleTPS);
        flywheel.update();
        // Manual control targetTPS
        if (gamepad2.dpad_up && !gamepad1.dpadUpWasPressed()) {
            targetTPS += 10;
        }
        if (gamepad2.dpad_down && !gamepad1.dpadDownWasPressed()) {
            targetTPS -= 10;
        }

        autoAimWithOdometry.dynamicTargetTPS();

        telemetry.addData("Target TPS", targetTPS);
        telemetry.addData("Actual TPS", flywheel.getVelocity());
        telemetry.addData("Goal X", autoAimWithOdometry.getBackdropPoseX());
        telemetry.addData("Goal Y", autoAimWithOdometry.getBackdropPoseY());
        telemetry.addData("Robot X", autoAimWithOdometry.getFollower().getPose().getX());
        telemetry.addData("Robot Y", autoAimWithOdometry.getFollower().getPose().getY());
        telemetry.addData("Robot H", Math.toDegrees(autoAimWithOdometry.getFollower().getPose().getHeading()));

        startingPose = new Pose(22, 120, 135);
        follower.setStartingPose(startingPose);
        follower.update();
    }
}
