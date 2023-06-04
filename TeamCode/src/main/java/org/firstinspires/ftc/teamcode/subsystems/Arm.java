package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
public class Arm {
    private DcMotor armMotor;
    static final double MOTOR_TICK_COUNT = 1425.1;
    private double quarterTurn = (double)MOTOR_TICK_COUNT/4;
    public Arm (HardwareMap hardwareMap) {

        armMotor = hardwareMap.dcMotor.get("arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);





    }
    //CRServoImplEx slide = hardwareMap.get(CRServoImplEx.class, "slide");

        public void armUp(){

            //int target = armMotor.getTargetPosition() + (int)quarterTurn;
            armMotor.setTargetPosition((int)quarterTurn);
            armMotor.setPower(.2);


        }
        public void armDown(){
        armMotor.setTargetPosition(-(int)quarterTurn);
        armMotor.setPower(.2);
        }
        public int getArmMotorTargetPosition(){
            return armMotor.getTargetPosition();
        }
        public double getArmMotorPower(){
            return armMotor.getPower();
        }
        public void setArmMotor(){
        armMotor.setPower(.5);
        }
        public void stopArmMotor(){
        armMotor.setPower(0);
        }


    }

