package com.team9889.ftc2024.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Arm {
    public DcMotorEx extend;
    public DcMotor arm;
    public Servo  rotate, claw;
//    CRServo intake2Right, intake2Left;
//    claw2;
//    intake2, intake2Right, intake2Left;
    CRServo intake3;

// hehehehe


    public void init(HardwareMap hardwareMap){
        arm = hardwareMap.dcMotor.get("arm");

        rotate = hardwareMap.servo.get("rotate");
//        claw2 = hardwareMap.crservo.get("claw2");
        claw = hardwareMap.servo.get("claw");

        extend = (DcMotorEx) hardwareMap.get(DcMotor.class,"extend");
        extend.setPositionPIDFCoefficients(15);
        extend.setTargetPosition(extend.getCurrentPosition());
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        extend.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(15, 0.05, 0, 0));



//        intake2 = hardwareMap.crservo.get("intake2");
//        intake3 = hardwareMap.crservo.get("intake3");
//
//        intake2Left = hardwareMap.crservo.get("intake2Left");
//        intake2Right = hardwareMap.crservo.get("intake2Right");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setExtencfdfdsionPower(double power){
        extend.setPower(power);
    }

    public void setArmRotation(double armPower){
        arm.setPower(armPower);


    }

//    public void setIntake3Power(double power){
//        intake3.setPower(power);
//    }


// teehee



    public void setClawPosition(double Claw){
        claw.setPosition(Claw);
    }

    public void setRotation(double Rotation){
        rotate.setPosition(Rotation);
    }

    public void setExtensionPower(double power){
        extend.setPower(power);
    }

//    public void setIntakePower(double intake2Power){
//        intake2.setPower(intake2Power);
//    }
//
//
//    public void setIntake2Power(double intake2Power){
//        intake2Right.setPower(intake2Power);
//        intake2Left.setPower(-intake2Power);
//    }

//
//


// *chuckles


}
