package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMUjeff {
    YawPitchRollAngles angle;
    AngularVelocity angularVelocity;
    IMU imu;
    public IMUjeff(HardwareMap hardwareMap, String name) {
        imu = hardwareMap.get(IMU.class, name);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        if (name.equals("cIMU")) {
            logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        }

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        angle = imu.getRobotYawPitchRollAngles();
    }

    public double getYaw(){
        angle = imu.getRobotYawPitchRollAngles();
        return angle.getYaw(AngleUnit.RADIANS);
    }

    public double getPitch() {
        angle = imu.getRobotYawPitchRollAngles();
        return angle.getPitch(AngleUnit.RADIANS);
    }

    public double getRoll() {
        angle = imu.getRobotYawPitchRollAngles();
        return angle.getRoll(AngleUnit.RADIANS);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void resetPitch() {
        imu.resetYaw();
    }

    public void resetRoll() {
        imu.resetYaw();
    }

    public double getYawVel() {
        angle = imu.getRobotYawPitchRollAngles();
        return angle.getPitch(AngleUnit.RADIANS);
    }
}
