    package org.firstinspires.ftc.teamcode.SocalCode.autonomous.AutoOpMode;

    import com.pedropathing.follower.Follower;
    import com.pedropathing.geometry.Pose;
    import com.pedropathing.util.Timer;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;

    import com.qualcomm.robotcore.util.ElapsedTime;
    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
    import org.firstinspires.ftc.teamcode.SocalCode.autonomous.Paths.optimalClosePaths;
    import org.firstinspires.ftc.teamcode.SocalCode.subsystems.FlywheelSystem;
    import org.firstinspires.ftc.teamcode.SocalCode.subsystems.DoubleIntake;

    @Autonomous(name = "Optimal Close BLUE", group = "Examples")
    public class optimalCloseBlue extends OpMode {

        private Follower follower;
        private Timer pathTimer, opmodeTimer;

        private int pathState;

        private optimalClosePaths paths;

        private boolean blueTeam = true;

        private double gateWait = 1000;
        private double pushGateWait = 0;

        FlywheelSystem flywheelSystem;
        DoubleIntake intake;
        private boolean shootingInitialized = false;
        ElapsedTime shotTimer = new ElapsedTime();
        ElapsedTime intakeTimer = new ElapsedTime();
        private static boolean runKickers = false;

        private boolean pathTimerReset = false;

        @Override
        public void init() {
            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();

            follower = Constants.createFollower(hardwareMap);
            paths = new optimalClosePaths(follower, blueTeam);
            follower.setStartingPose(new Pose(20.570, 122.064, Math.toRadians(135)));

            flywheelSystem = new FlywheelSystem(hardwareMap);
            intake = new DoubleIntake(hardwareMap);
        }

        // Update the auto pathing so it uses the new paths from optimalClosePaths. They paths sshould be placed between 4,5 and 6,7 like how it is named.
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
                        follower.setMaxPower(1);
                        intake.setAutoIntakeState(DoubleIntake.autoIntakeState.INTAKE);
                        follower.followPath(paths.Path3);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (!follower.isBusy() && handleShooting()) {
                        intake.setAutoIntakeState(DoubleIntake.autoIntakeState.INTAKE); // was idle
                        follower.followPath(paths.Path4);
                        follower.setMaxPower(1);
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (!follower.isBusy()) {
                        // Start intake immediately upon arrival
                        intake.setAutoIntakeState(DoubleIntake.autoIntakeState.INTAKE);

                        if (!pathTimerReset) {
                            pathTimer.resetTimer();
                            pathTimerReset = true;
                        }

//                        if (pathTimer.getElapsedTime() > pushGateWait) {
                            pathTimerReset = false; // Reset ONLY when moving to next state
                            follower.followPath(paths.Path45);
                            setPathState(45);
//                        }
                    }
                    break;
                case 45:
                    if (!follower.isBusy()) {
                        if (!pathTimerReset) {
                            pathTimer.resetTimer();
                            pathTimerReset = true;
                        }

                        if (pathTimer.getElapsedTime() > gateWait) {
                            pathTimerReset = false;
                            intake.setAutoIntakeState(DoubleIntake.autoIntakeState.IDLE);
                            follower.setMaxPower(1);
                            follower.followPath(paths.Path5);
                            setPathState(5);
                        }
                    }
                    break;
                case 5:
                    if (!follower.isBusy() && handleShooting()) {
                        intake.setAutoIntakeState(DoubleIntake.autoIntakeState.INTAKE); // was idle
                        follower.followPath(paths.Path6);
                        follower.setMaxPower(1);
                        setPathState(6);
                    }
                    break;
                case 6:
                    if (!follower.isBusy()) {
                        intake.setAutoIntakeState(DoubleIntake.autoIntakeState.INTAKE);

                        if (!pathTimerReset) {
                            pathTimer.resetTimer();
                            pathTimerReset = true;
                        }

//                        if (pathTimer.getElapsedTime() > pushGateWait) {
                            pathTimerReset = false;
                            follower.followPath(paths.Path67);
                            setPathState(67); // Fixed: Go to 67, not 7
//                        }
                    }
                    break;
                case 67:
                    if (!follower.isBusy()) {
                        if (!pathTimerReset) {
                            pathTimer.resetTimer();
                            pathTimerReset = true;
                        }

                        if (pathTimer.getElapsedTime() > gateWait) {
                            pathTimerReset = false;
                            intake.setAutoIntakeState(DoubleIntake.autoIntakeState.IDLE);
                            follower.setMaxPower(1);
                            follower.followPath(paths.Path7);
                            setPathState(7);
                        }
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
                        intake.setAutoIntakeState(DoubleIntake.autoIntakeState.INTAKE);
                        follower.setMaxPower(1);
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
                        setPathState(-1);
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

            if (shotTimer.milliseconds() < 2000) { // If timing is a problem could be this line
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

