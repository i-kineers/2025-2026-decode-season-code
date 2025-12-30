package org.firstinspires.ftc.teamcode.autonomous.AutoOpMode;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.autonomous.Paths.testPathCloseBlue;
import org.firstinspires.ftc.teamcode.subsystems.DoubleMotorOuttakePID;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name = "testCloseBlue", group = "Examples")
public class testCloseBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    Intake intake;
    DoubleMotorOuttakePID outtake;

    private int pathState;

    private testPathCloseBlue paths;

//    DoubleMotorOuttakePID shooter;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        intake = new Intake(hardwareMap);
        outtake = new DoubleMotorOuttakePID(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        paths = new testPathCloseBlue(follower);
        follower.setStartingPose(new Pose(22, 120, Math.toRadians(135)));
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()){
                    outtake.autoRapidShoot(2600,3500, 500);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    intake.autoIntakeOn();
                    follower.followPath(paths.Path2);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intake.autoIntakeOff();
                    sleep(1000);
                    follower.setMaxPower(1);
                    follower.followPath(paths.Path3);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    intake.autoIntakeOn();
                    sleep(1000);
                    intake.autoIntakeOff();
                    outtake.autoRapidShoot(2600, 3500, 500);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
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
