package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.CustomCurve;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import java.util.Arrays;

public class LinearSpline extends CustomCurve {
    private Pose startPose;
    private Pose endPose;
    private Pose diffPose;

    public LinearSpline(Pose startPoint, Pose endPoint) {
        super(startPoint, endPoint);
    }

    public LinearSpline(FuturePose startPoint, FuturePose endPoint) {
        super(startPoint, endPoint);
    }

    public LinearSpline(Pose startPoint, Pose endPoint, PathConstraints constraints) {
        super(Arrays.asList(startPoint, endPoint), constraints);
    }

    @Override
    public String pathType() {
        return "Linear Spline";
    }

    @Override
    public LinearSpline getReversed() {
        LinearSpline curve = new LinearSpline(getControlPoints().get(1), getControlPoints().get(0), this.getPathConstraints());
        curve.initialize();
        return curve;
    }

    @Override
    public Vector getDerivative(double t) {
        return diffPose.getAsVector();
    }

    @Override
    public Pose getPose(double t) {
        if (startPose == null) init();

        double x = t * diffPose.getX() + startPose.getX();
        double y = t * diffPose.getY() + startPose.getY();

        return new Pose(x, y);
    }

    public void init() {
        startPose = getControlPoints().get(0);
        endPose = getControlPoints().get(1);
        diffPose = endPose.minus(startPose);
    }

    @Override
    public Vector getSecondDerivative(double t) {
        return new Vector();
    }
}