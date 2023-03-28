package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import org.firstinspires.ftc.teamcode.subsystems.ControllerFeatures;

@TeleOp(name="FieldCentricNoDriveSubSystem", group="DriveModes")
public class FieldCentricNoDriveSubSystem extends LinearOpMode{

    // this changes the speed multiplier for wheels
    int mult = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor FL = hardwareMap.dcMotor.get("front left");
        DcMotor BL = hardwareMap.dcMotor.get("back left");
        DcMotor FR = hardwareMap.dcMotor.get("front right");
        DcMotor BR = hardwareMap.dcMotor.get("back right");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        // imports the IMU
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            // sets controller colors- find in Subsystem ControllerLights
            ControllerFeatures feature = new ControllerFeatures();
            feature.setRainbow();

            // imu reset is dpad up
            if (gamepad1.dpad_up) {
                imu.initialize(parameters);
            }

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            // the 1 * is for correcting drift btw
            // variable mult is for the speed multiplier of the motors

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (1 * (rotY + rotX + rx)) / denominator;
            double backLeftPower = (1 * (rotY - rotX + rx)) / denominator;
            double frontRightPower = (1 * (rotY - rotX - rx)) / denominator;
            double backRightPower = (1 * (rotY + rotX - rx)) / denominator;

            // speed is default set to 75%
            // controllers will rumble and speed will be set to max if right trigger is held
            // controllers will also change lights depending on what trigger is held




            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);

        }

    }

}
