package org.firstinspires.ftc.teamcode.SocalCode.autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class optimalClosePaths {

    // Heading angles are set to favor blue
    private double startHeading = Math.toRadians(135); //DONE
    private double gatePickUpHeading = Math.toRadians(150); // DONE
    private double gateBackupPickupHeading = Math.toRadians(135); // DONE
    private double shootHeading = Math.toRadians(225); // DONE
    private double pickUpHeading = Math.toRadians(180); // DONE
    private double angledPickUpHeading = Math.toRadians(200);
    private double resetHeading = Math.toRadians(180); // DONE

    // Paths will be set to favor blue
    private Pose startPose = new Pose(20.570, 122.064); // DONE
    private Pose shootPose = new Pose(55.315,88.219); // DONE
    private Pose homePose = new Pose(58, 112); // DONE
    private Pose gatePose = new Pose(11.598, 60.312); // DONE


    private Pose spike1 = new Pose(16.605, 84.386); // NEEDS TO BE REDONE
    private Pose spike2 = new Pose(8.356, 55.855); // DONE
    private Pose gatePickupPose = new Pose(11.598, 60.312); // DONE
    private Pose backupGatePickupPose = new Pose(10.550, 55); // DONE

    private Pose spike2Control = new Pose(78.55, 60.830); // DONE
    private Pose returnSpike2Control = new Pose(43.932, 64.0425); // DONE
    private Pose gateControl = new Pose(36.415, 67.306); // DONE
    private Pose returnGateControl = new Pose(45.8861, 68.074); // DONE
    private Pose spike1Control = new Pose(34.960, 84.302); // NEEDS TO BE REDONE

    // PathChain member variables, to be initialized in the constructor
    public PathChain Path1, Path2, Path3, Path4, Path45, Path5, Path6, Path67, Path7, Path8, Path9, Path10;

    public optimalClosePaths(Follower follower, boolean teamColor) {

        // Check if team color is red to reverse coordinates
        if (!teamColor) {
            shootHeading = reflect(shootHeading);
            pickUpHeading = reflect(pickUpHeading);
            resetHeading = reflect(resetHeading);
            startPose = reflect(startPose);
            shootPose = reflect(shootPose);
            gatePose = reflect(gatePose);
            spike2 = reflect(spike2);
            spike1 = reflect(spike1);
            spike2Control = reflect(spike2Control);
            returnSpike2Control = reflect(returnSpike2Control);
            gateControl = reflect(gateControl);
            gateControl = reflect(gateControl);
            homePose = reflect(homePose);
            gatePickupPose = reflect(gatePickupPose);
            spike1Control = reflect(spike1Control);
            gateBackupPickupHeading = reflect(gateBackupPickupHeading);
            returnGateControl = reflect(returnGateControl);
        }


        Path1 = follower.pathBuilder().addPath( // DONE
                        new BezierLine(
                                startPose,

                                shootPose
                        )
                ).setLinearHeadingInterpolation(startHeading, shootHeading)

                .build();

        Path2 = follower.pathBuilder().addPath( // DONE
                        new BezierCurve(
                                shootPose,
                                spike2Control,
                                spike2
                        )
                )
                .addParametricCallback(0.35, () -> follower.setMaxPower(0.3))
//                .setConstantHeadingInterpolation(0)
                .setLinearHeadingInterpolation(shootHeading, pickUpHeading, 0.5)

                .build();

        Path3 = follower.pathBuilder().addPath( // DONE
                        new BezierCurve(
                                spike2,
                                returnSpike2Control,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(pickUpHeading, shootHeading)

                .build();

        Path4 = follower.pathBuilder().addPath( // DONE
                        new BezierCurve(
                                shootPose,
                                gateControl,
                                gatePickupPose
                        )
                ).setLinearHeadingInterpolation(shootHeading, gatePickUpHeading)
                .addParametricCallback(0.8, () -> follower.setMaxPower(0.6))
                .build();

        Path45 = follower.pathBuilder().addPath( // DONE
                        new BezierLine(
                                gatePickupPose,
                                backupGatePickupPose
                        )
                ).setLinearHeadingInterpolation(gatePickUpHeading, gateBackupPickupHeading)
                .build();


        Path5 = follower.pathBuilder().addPath( // DONE
                        new BezierCurve(
                                backupGatePickupPose,
                                returnGateControl,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(gateBackupPickupHeading, shootHeading)
                .build();

        Path6 = follower.pathBuilder().addPath( // DONE
                        new BezierCurve(
                                shootPose,
                                gateControl,
                                gatePickupPose
                        )
                ).setLinearHeadingInterpolation(shootHeading, gatePickUpHeading)
                .addParametricCallback(0.8, () -> follower.setMaxPower(0.6))

                .build();

        Path67 = follower.pathBuilder().addPath( // DONE
                        new BezierLine(
                                gatePickupPose,
                                backupGatePickupPose
                        )
                ).setLinearHeadingInterpolation(gatePickUpHeading, gateBackupPickupHeading)

                .build();

        Path7 = follower.pathBuilder().addPath( // DONE
                        new BezierCurve(
                                backupGatePickupPose,
                                returnGateControl,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(gateBackupPickupHeading, shootHeading)

                .build();

        Path8 = follower.pathBuilder().addPath( // NOT DONE
                        new BezierCurve(
                                shootPose,
                                spike1Control,
                                spike1
                        )
                ).addParametricCallback(0.3, () -> follower.setMaxPower(0.5))
                .setLinearHeadingInterpolation(shootHeading, pickUpHeading, 0.8)

                .build();

        Path9 = follower.pathBuilder().addPath( // NOT DONE
                        new BezierLine(
                                spike1,

                                shootPose
                        )
                ).setLinearHeadingInterpolation(pickUpHeading, shootHeading)

                .build();

        Path10 = follower.pathBuilder().addPath( // NOT DONE
                        new BezierLine(
                                shootPose,

                                homePose
                        )
                ).setLinearHeadingInterpolation(shootHeading, pickUpHeading)

                .build();
    }

    private Pose reflect(Pose pose) {
        // We pass 0 for heading here because the heading is reflected separately.
        return new Pose(144 - pose.getX(), pose.getY(), 0);
    }

    private double reflect(double angleInRadians) {
        return Math.PI - angleInRadians;
    }

}
