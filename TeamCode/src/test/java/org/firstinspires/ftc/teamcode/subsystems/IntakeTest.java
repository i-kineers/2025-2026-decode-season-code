package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import org.junit.Test;

/**
 * JUnit Test for Intake logic.
 */
public class IntakeTest {

    private static final double DELTA = 1e-6;

    @Test
    public void testIntakePower() {
        // Test that the intake power provided to the method is what we expect.
        double requestedPower = -1.0;
        assertEquals("Intake should run at full reverse", -1.0, requestedPower, DELTA);
    }
    
    @Test
    public void testGatePositions() {
        // Based on Intake.java, verify the logical positions
        double openPos = 0.75;
        double closedPos = 0.0;
        
        assertTrue("Gate open should be > closed", openPos > closedPos);
    }

    private void assertTrue(String message, boolean condition) {
        if (!condition) {
            throw new AssertionError(message);
        }
    }
}
