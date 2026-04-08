package org.firstinspires.ftc.teamcode.SocalCode.autonomous.AutoOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.SocalCode.autonomous.Paths.farAutoPaths;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.FlywheelSystem;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.DoubleIntake;

@Autonomous(name = "Far Auto SYMBIOTIC RED", group = "Autonomous")
public class farAutoRed extends OpMode {

    private Follower follower;
    private Timer pathTimer;

    private int pathState;
    private farAutoPaths paths;

    // --- RED ALLIANCE SETTINGS ---
    private boolean blueTeam = false;
    private boolean useSpike = true;

    FlywheelSystem flywheelSystem;
    DoubleIntake intake;

    private boolean shootingInitialized = false;
    private boolean runKickers = false;
    ElapsedTime shotTimer = new ElapsedTime();

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // Pass 'false' for isBlueSide to trigger the reflect() logic in farAutoPaths
        paths = new farAutoPaths(follower, false, useSpike);

        // REFLECTED STARTING POSE:
        // X: 144 - 64 = 80
        // Y: 8
        // Heading: Math.PI - Math.toRadians(180) = 0
        follower.setStartingPose(new Pose(80.000, 8.000, Math.toRadians(180)));

        flywheelSystem = new FlywheelSystem(hardwareMap);
        intake = new DoubleIntake(hardwareMap);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start to Shoot 1
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path1);
                    flywheelSystem.setTargetTPS(1450);
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
                    follower.followPath(paths.Path9);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy() && handleShooting()) {
                    intake.setAutoIntakeState(DoubleIntake.autoIntakeState.IDLE);
                    follower.followPath(paths.PathPark);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    flywheelSystem.setTargetTPS(0);
                    setPathState(-1);
                }
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private boolean handleShooting() {
        if (!shootingInitialized) {
            shotTimer.reset();
            intake.setAutoIntakeState(DoubleIntake.autoIntakeState.SHOOTING);
            shootingInitialized = true;
        }

        if (shotTimer.milliseconds() < 2000) {
            runKickers = true;
            return false;
        } else {
            runKickers = false;
            shootingInitialized = false;
            return true;
        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        flywheelSystem.autoShootLogic(runKickers);
        intake.autoIntakeOn(blueTeam); // blueTeam is false here, so it uses Red intake logic

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }
}