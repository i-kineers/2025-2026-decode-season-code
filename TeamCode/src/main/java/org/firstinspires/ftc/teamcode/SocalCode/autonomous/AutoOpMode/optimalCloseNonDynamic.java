package org.firstinspires.ftc.teamcode.SocalCode.autonomous.AutoOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.CodePriorILT.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.SocalCode.autonomous.Paths.optimalClosePaths;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.FlywheelSystem;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.DoubleIntake;

@Autonomous(name = "OptimalCloseNonDynamic", group = "Examples")
public class optimalCloseNonDynamic extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    private optimalClosePaths paths;

    private boolean blueTeam = true;

    FlywheelSystem flywheelSystem;
    DoubleIntake intake;

    private boolean shootingInitialized = false;
    ElapsedTime shotTimer = new ElapsedTime();
    private static boolean runKickers = false;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        paths = new optimalClosePaths(follower, blueTeam);
        follower.setStartingPose(new Pose(19.5, 122.6, Math.toRadians(135)));

        flywheelSystem = new FlywheelSystem(hardwareMap);
        intake = new DoubleIntake(hardwareMap);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path1);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy() && handleShooting()) {
                    intake.setAutoIntakeState(DoubleIntake.autoIntakeState.INTAKE);
                    follower.followPath(paths.Path2);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    intake.setAutoIntakeState(DoubleIntake.autoIntakeState.IDLE);
                    follower.followPath(paths.Path3);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && handleShooting()) {
                    intake.setAutoIntakeState(DoubleIntake.autoIntakeState.INTAKE);
                    follower.followPath(paths.Path4);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    intake.setAutoIntakeState(DoubleIntake.autoIntakeState.IDLE);
                    follower.followPath(paths.Path5);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && handleShooting()) {
                    intake.setAutoIntakeState(DoubleIntake.autoIntakeState.INTAKE);
                    follower.followPath(paths.Path6);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    intake.setAutoIntakeState(DoubleIntake.autoIntakeState.IDLE);
                    follower.followPath(paths.Path7);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && handleShooting()) {
                    intake.setAutoIntakeState(DoubleIntake.autoIntakeState.INTAKE);
                    follower.followPath(paths.Path8);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    intake.setAutoIntakeState(DoubleIntake.autoIntakeState.IDLE);
                    follower.followPath(paths.Path9);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && handleShooting()) {
                    intake.setAutoIntakeState(DoubleIntake.autoIntakeState.IDLE);
                    follower.followPath(paths.Path10);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    flywheelSystem.setTargetTPS(0);
                    setPathState(2);
                }
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        flywheelSystem.autoShootLogic(runKickers);
        intake.autoIntakeOn(blueTeam);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private boolean handleShooting() {
        if (!shootingInitialized) {
            shotTimer.reset();
            intake.setAutoIntakeState(DoubleIntake.autoIntakeState.SHOOTING);
            shootingInitialized = true;
        }

        if (shotTimer.milliseconds() < 2500) { // If timing is a problem could be this line
            runKickers = true;
            return false;
        } else {
            runKickers = false;
            shootingInitialized = false;
            return true;
        }
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
