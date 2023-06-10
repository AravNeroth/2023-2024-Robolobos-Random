package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private DcMotor armMotor;
    static final double MOTOR_TICK_COUNT = 1425.1;
    private double quarterTurn = (double)MOTOR_TICK_COUNT/4;
    private double fifthTurn = (double)MOTOR_TICK_COUNT/5;
    private double sixthTurn = (double)MOTOR_TICK_COUNT/6;
    private double eighthTurn= (double)MOTOR_TICK_COUNT/8;
    public Arm (HardwareMap hardwareMap) {

        armMotor = hardwareMap.dcMotor.get("arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);





    }
    //CRServoImplEx slide = hardwareMap.get(CRServoImplEx.class, "slide");

        public void armUp() {

            //int target = armMotor.getTargetPosition() + (int)quarterTurn;
            armMotor.setTargetPosition((int) sixthTurn);
            armMotor.setPower(.3);
        }
        public void armMid(){
        armMotor.setTargetPosition((int)eighthTurn);
        armMotor.setPower(.3);
        }
            public void armDown() {
                armMotor.setTargetPosition(-(int) eighthTurn + 240);
                armMotor.setPower(.3);
            }
            public int getArmMotorTargetPosition(){

                return armMotor.getTargetPosition();
            }
            public double getArmMotorPower(){

                return armMotor.getPower();
            }


        }
