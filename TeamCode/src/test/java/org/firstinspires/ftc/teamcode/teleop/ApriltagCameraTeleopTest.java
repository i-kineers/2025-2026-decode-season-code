package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.junit.MockitoJUnitRunner;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

/**
 * Unit tests for the ApriltagCameraTeleop class.
 * This class focuses on verifying the logic within the teleop opmode.
 */
@RunWith(MockitoJUnitRunner.class)
public class ApriltagCameraTeleopTest {

    // Mock the dependencies of ApriltagCameraTeleop
    @Mock
    private Camera mockCamera;
    @Mock
    private FieldCentricDrive mockDrive;

    // Use a real Gamepad object that we can manipulate in tests
    private Gamepad gamepad1;

    // This is the class we are testing
    private ApriltagCameraTeleop apriltagCameraTeleop;

    @Before
    public void setUp() {
        // Before each test, create a fresh Gamepad object
        gamepad1 = new Gamepad();

        // Instantiate the class under test using the test-specific constructor,
        // injecting our mock objects.
        apriltagCameraTeleop = new ApriltagCameraTeleop(mockCamera, mockDrive);

        // The OpMode class has a 'gamepad1' field, which we need to set manually for our test
        apriltagCameraTeleop.gamepad1 = gamepad1;

        // The OpMode class has a 'telemetry' field. We'll give it a real Telemetry implementation.
        apriltagCameraTeleop.telemetry = new TelemetryImpl(mock(OpMode.class));

        // We still call init() to complete the setup, but it will now use our injected mocks.
        apriltagCameraTeleop.init();
    }

    /**
     * Test to ensure the robot drives manually using joystick inputs
     * when no other buttons are pressed.
     */
    @Test
    public void loop_whenNoButtonsPressed_shouldDriveManually() {
        // Arrange: Set specific values for the gamepad joysticks.
        // Use values that are perfectly representable in binary to avoid floating point errors.
        gamepad1.left_stick_y = -0.5f;   // -1/2
        gamepad1.left_stick_x = 0.25f;  //  1/4
        gamepad1.right_stick_x = 0.125f; //  1/8

        // Act: Run the main loop of the opmode
        apriltagCameraTeleop.loop();

        // Assert: Verify that the drive system's 'drive' method was called
        // with the exact values from the joysticks.
        verify(mockDrive).drive(-0.5, 0.25, 0.125);
    }

    /**
     * Test to verify that when the 'x' button is pressed and an AprilTag is detected,
     * the robot attempts to align with the tag and ignores joystick input.
     */
    @Test
    public void loop_whenXIsPressedAndTagIsDetected_shouldAlignToTag() {
        // Arrange: Simulate the 'x' button being pressed
        gamepad1.x = true;
        // Define the expected turn power that the camera will provide
        Double expectedTurnPower = 0.3;
        // Configure the mock camera to return the turn power when asked
        when(mockCamera.useBearingToAlign()).thenReturn(expectedTurnPower);

        // Act: Run the main loop
        apriltagCameraTeleop.loop();

        // Assert: Verify that the drive system was instructed to turn with the
        // calculated power, and that forward/strafe movements are zero.
        verify(mockDrive).drive(0, 0, expectedTurnPower);
    }

    /**
     * Test to verify that when 'x' is pressed and no tag is found, the robot stops completely.
     */
    @Test
    public void loop_whenXIsPressedAndNoTagIsDetected_shouldStopMoving() {
        // Arrange: Simulate the 'x' button being pressed
        gamepad1.x = true;
        // Set joystick values to prove they are being ignored
        gamepad1.left_stick_y = -0.5f;
        gamepad1.left_stick_x = 0.25f;
        gamepad1.right_stick_x = 0.125f;
        // Configure the mock camera to return 'null', simulating no tag found
        when(mockCamera.useBearingToAlign()).thenReturn(null);

        // Act: Run the main loop
        apriltagCameraTeleop.loop();

        // Assert: Verify that the drive system is instructed to stop all movement (0, 0, 0)
        // because no tag was found during the alignment attempt.
        verify(mockDrive).drive(0, 0, 0);
    }

    /**
     * Test to ensure the camera stream is stopped when dpad_down is pressed.
     */
    @Test
    public void loop_whenDpadDownIsPressed_shouldStopCameraStream() {
        // Arrange: Simulate the dpad_down button being pressed
        gamepad1.dpad_down = true;

        // Act: Run the main loop
        apriltagCameraTeleop.loop();

        // Assert: Verify that the camera's 'stopStream' method was called.
        verify(mockCamera).stopStream();
        // Also ensure the 'resumeStream' method was NOT called.
        verify(mockCamera, never()).resumeStream();
    }

    /**
     * Test to ensure the camera stream is resumed when dpad_up is pressed.
     */
    @Test
    public void loop_whenDpadUpIsPressed_shouldResumeCameraStream() {
        // Arrange: Simulate the dpad_up button being pressed
        gamepad1.dpad_up = true;

        // Act: Run the main loop
        apriltagCameraTeleop.loop();

        // Assert: Verify that the camera's 'resumeStream' method was called.
        verify(mockCamera).resumeStream();
        // Also ensure the 'stopStream' method was NOT called.
        verify(mockCamera, never()).stopStream();
    }
}
