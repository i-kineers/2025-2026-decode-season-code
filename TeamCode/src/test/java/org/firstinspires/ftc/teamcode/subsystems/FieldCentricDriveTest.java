package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * JUnit Test for FieldCentricDrive logic.
 * These tests run on your PC without the robot.
 * To run: Click the green play icon in the gutter next to the class name.
 */
public class FieldCentricDriveTest {

    private static final double DELTA = 1e-6;

    @Test
    public void testForward() {
        // Input: Stick pushed up (leftStickY = -1.0), Heading 0
        // Expected: All motors 1.0
        double[] powers = FieldCentricDrive.calculatePowers(-1.0, 0.0, 0.0, 0.0);
        
        assertEquals("Front Left Forward", 1.0, powers[0], DELTA);
        assertEquals("Back Left Forward", 1.0, powers[1], DELTA);
        assertEquals("Front Right Forward", 1.0, powers[2], DELTA);
        assertEquals("Back Right Forward", 1.0, powers[3], DELTA);
    }

    @Test
    public void testStrafeRight() {
        // Input: Stick pushed right (leftStickX = 1.0), Heading 0
        // Expected: FL+, BL-, FR-, BR+
        double[] powers = FieldCentricDrive.calculatePowers(0.0, 1.0, 0.0, 0.0);
        
        assertTrue("Front Left Strafe Right", powers[0] > 0);
        assertTrue("Back Left Strafe Right", powers[1] < 0);
        assertTrue("Front Right Strafe Right", powers[2] < 0);
        assertTrue("Back Right Strafe Right", powers[3] > 0);
    }

    @Test
    public void testFieldCentricity90Deg() {
        // Input: Stick pushed forward, but robot is rotated 90 deg clockwise (-PI/2)
        // From a field perspective, "Forward" is now "To the Robot's Left"
        // Expected: Robot should Strafe Left relative to its chassis
        double heading = Math.toRadians(-90);
        double[] powers = FieldCentricDrive.calculatePowers(-1.0, 0.0, 0.0, heading);
        
        // Strafe Left Signature: FL-, BL+, FR+, BR-
        assertTrue("Should Strafe Left (FL)", powers[0] < 0);
        assertTrue("Should Strafe Left (BL)", powers[1] > 0);
        assertTrue("Should Strafe Left (FR)", powers[2] > 0);
        assertTrue("Should Strafe Left (BR)", powers[3] < 0);
    }

    @Test
    public void testNormalization() {
        // Input: Full Forward, Full Strafe, Full Turn
        // Math would result in powers > 3.0, logic must scale them to 1.0
        double[] powers = FieldCentricDrive.calculatePowers(-1.0, 1.0, 1.0, 0.0);
        
        for (int i = 0; i < 4; i++) {
            assertTrue("Power " + i + " must be <= 1.0", Math.abs(powers[i]) <= 1.000001);
        }
    }
}
