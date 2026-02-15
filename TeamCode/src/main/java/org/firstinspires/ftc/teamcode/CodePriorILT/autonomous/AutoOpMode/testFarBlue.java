package org.firstinspires.ftc.teamcode.CodePriorILT.autonomous.AutoOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.CodePriorILT.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.CodePriorILT.autonomous.Paths.testPathFarBlue;
import org.firstinspires.ftc.teamcode.CodePriorILT.subsystems.FlywheelSystem;
import org.firstinspires.ftc.teamcode.CodePriorILT.subsystems.Intake;

@Autonomous(name = "testFarBlue", group = "Examples")
public class testFarBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    private testPathFarBlue paths;

    FlywheelSystem flywheelSystem;
    Intake intake;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        paths = new testPathFarBlue(follower);
        follower.setStartingPose(new Pose(55.273, 7.516, Math.toRadians(90)));

        flywheelSystem = new FlywheelSystem(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // 1. Start the path ONCE
                follower.followPath(paths.Path1);
                setPathState(1); // Immediately move to the next state to wait
                break;

            case 1:
                // 2. Wait until Path 1 is done
                if (!follower.isBusy()) {
                    // 3. Perform action
                    flywheelSystem.autoRapidShoot(1600, 3000, 500);
                    setPathState(2);
                }
                break;

            case 2:
                // 4. Start Path 2 ONCE
                follower.followPath(paths.Path2);
                setPathState(3);
                break;

            case 3:
                // 5. Wait for Path 2 to finish
                if (!follower.isBusy()) {
                    setPathState(-1); // Finished
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }

    public void autoShoot() {

    }
}
