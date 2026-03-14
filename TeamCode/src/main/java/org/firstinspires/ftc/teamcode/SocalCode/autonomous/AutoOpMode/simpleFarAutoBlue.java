package org.firstinspires.ftc.teamcode.SocalCode.autonomous.AutoOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.SocalCode.autonomous.Paths.SimpleFarPath;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.FlywheelSystem;
import org.firstinspires.ftc.teamcode.SocalCode.subsystems.DoubleIntake;

@Autonomous(name = "SimpleFarAutoBlue", group = "Autonomous")
public class simpleFarAutoBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;
    private SimpleFarPath paths;

    // Default to true as per your structure
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
        paths = new SimpleFarPath(follower);

        // Starting pose matches the start of your Path1: (54.833, 5.799)
        // Heading set to 180 degrees (matching your interpolation start)
        follower.setStartingPose(new Pose(54.833, 5.799, Math.toRadians(180)));

        flywheelSystem = new FlywheelSystem(hardwareMap);
        intake = new DoubleIntake(hardwareMap);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Move from Start to Pose(64, 15)
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path1);
                    setPathState(1);
                }
                break;

            case 1:
                // Once at the end of Path 1, run shooting logic
                if (!follower.isBusy() && handleShooting()) {
                    intake.setAutoIntakeState(DoubleIntake.autoIntakeState.INTAKE);
                    follower.followPath(paths.Path2);
                    setPathState(2);
                }
                break;

            case 2:
                // Move from Pose(64, 15) to Pose(44, 15)
                if (!follower.isBusy()) {
                    // Final state logic
                    intake.setAutoIntakeState(DoubleIntake.autoIntakeState.IDLE);
                    flywheelSystem.setTargetTPS(0);
                    setPathState(-1); // End of Auto
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
        flywheelSystem.setTargetTPS(1450);
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
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}