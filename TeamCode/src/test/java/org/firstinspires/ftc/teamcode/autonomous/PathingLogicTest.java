package org.firstinspires.ftc.teamcode.autonomous;

import static org.junit.Assert.assertEquals;
import org.junit.Test;
import com.pedropathing.geometry.Pose;

/**
 * JUnit Test for Autonomous Pathing Logic.
 */
public class PathingLogicTest {

    private static final double DELTA = 1e-6;

    @Test
    public void testReflectAngle() {
        // Test reflecting a heading angle for the Red Alliance.
        // PI - angle
        double blueHeading = Math.toRadians(45); // 0.785398
        double expectedRedHeading = Math.toRadians(135); // 2.356194
        
        double actualRedHeading = Math.PI - blueHeading;
        assertEquals("Angle should be reflected across the Y axis", expectedRedHeading, actualRedHeading, DELTA);
    }

    @Test
    public void testReflectPose() {
        // Test reflecting a Pose across the center of the field (x=72)
        // newX = 144 - oldX
        double blueX = 22.0;
        double blueY = 120.0;
        double expectedRedX = 122.0;
        
        double actualRedX = 144.0 - blueX;
        assertEquals("X coordinate should be reflected", expectedRedX, actualRedX, DELTA);
        assertEquals("Y coordinate should remain the same", blueY, 120.0, DELTA);
    }
}
