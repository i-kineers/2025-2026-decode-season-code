package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasicOuttake {
    private DcMotor launcher;
    private CRServo sideLauncher1;
    private CRServo sideLauncher2;

    public BasicOuttake(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotor.class, "launcher"); // Initialize the member variable
        sideLauncher1 = hardwareMap.get(CRServo.class, "SideLauncher1"); // Initialize the member variable
        sideLauncher2 = hardwareMap.get(CRServo.class, "SideLauncher2"); // Initialize the member variable
    }

    public void setLaunchPower(double power) {
        launcher.setPower(power);
    }

    public void setSideLaunchPower(double power) {
        sideLauncher1.setPower(power);
        sideLauncher2.setPower(power);
    }

    public void emergencyStop() {
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setSideLaunchPower(0);
    }

    public int getLauncherPosition() {
        return launcher.getCurrentPosition();
    }
}
