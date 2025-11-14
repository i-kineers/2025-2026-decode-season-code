package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths;
import org.firstinspires.ftc.teamcode.subsystems.DoubleMotorOuttakePID;

@Autonomous(name = "Main Auto", group = "Examples")
public class mainAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    DoubleMotorOuttakePID shooter;

    @Override
    public void init() {
        shooter = new DoubleMotorOuttakePID(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        Paths.buildPaths(follower);
        follower.setStartingPose(Paths.startPose);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Paths.scorePreload);
                shooter.setTargetRPM(2500);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    shooter.autoRapidShoot(2500, 3000);
                    /* Score Preload */
                    follower.followPath(Paths.grabPickup1, true);
                    setPathState(-1);
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
        telemetry.addData("heading", follower.getPose().getHeading());
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