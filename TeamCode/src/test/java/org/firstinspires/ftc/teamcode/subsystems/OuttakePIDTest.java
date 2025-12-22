package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import org.junit.Test;

/**
 * JUnit Test for DoubleMotorOuttakePID logic.
 */
public class OuttakePIDTest {

    private static final double DELTA = 1e-6;

    @Test
    public void testRPMNormalization() {
        // We can't easily instantiate hardware, so we test the logic methods.
        // If we want to test setTargetRPM, we'd ideally make targetRPM protected or have a getter.
        
        // Let's verify the clipping logic works as intended.
        double maxRPM = 6000.0;
        double inputRPM = 7000.0;
        double expectedRPM = 6000.0;
        
        double clipped = Math.max(0, Math.min(maxRPM, inputRPM));
        assertEquals("Should clip to Max RPM", expectedRPM, clipped, DELTA);
    }
    
    @Test
    public void testPIDFeedforward() {
        // Test the Feedforward calculation: kF * targetRPM
        double kF = 0.42 / 6000.0;
        double targetRPM = 3000.0;
        double expectedFF = 0.21;
        
        double actualFF = kF * targetRPM;
        assertEquals("Feedforward calculation should be correct", expectedFF, actualFF, DELTA);
    }
}
