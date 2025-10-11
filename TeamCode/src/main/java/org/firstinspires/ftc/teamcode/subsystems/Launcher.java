package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Launcher {
    /* Initialize standard Hardware interfaces */
    /* Initialize standard Hardware interfaces */


    private final DcMotor Launcher;


    public Launcher(HardwareMap hardwareMap) {
        Launcher = hardwareMap.get(DcMotor.class, "launcher"); // Initialize the member variable

    }


    public void setLaunchPower(double power) {

        Launcher.setPower(power);
    }
}
