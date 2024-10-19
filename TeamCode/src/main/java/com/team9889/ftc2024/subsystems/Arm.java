package com.team9889.ftc2024.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Arm {
    public DcMotor arm;
    public Servo  claw ,rotate, extend;
//    CRServo claw2;
//    intake2, intake2Right, intake2Left;
    CRServo intake3;




    public void init(HardwareMap hardwareMap){
        arm = hardwareMap.dcMotor.get("arm");

        rotate = hardwareMap.servo.get("rotate");
        extend = hardwareMap.servo.get("extend");
//        claw2 = hardwareMap.crservo.get("claw2");
        claw = hardwareMap.servo.get("claw");

//        intake2 = hardwareMap.crservo.get("intake2");
        intake3 = hardwareMap.crservo.get("intake3");
//
//        intake2Left = hardwareMap.crservo.get("intake2Left");
//        intake2Right = hardwareMap.crservo.get("intake2Right");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);







    }
    public void setArmRotation(double armPower){
        arm.setPower(armPower);


    }

    public void setIntake3Power(double power){
        intake3.setPower(power);
    }




    public void setExtetion(double Extendo){
        extend.setPosition(Extendo);
    }

//    public void setClawPosition(double clawPos){
//        claw2.setPower(clawPos);
//    }

    public void setClawPosition(double Claw){
        claw.setPosition(Claw);
    }

    public void setRotation(double Rotation){
        rotate.setPosition(Rotation);
    }

//    public void setIntakePower(double intake2Power){
//        intake2.setPower(intake2Power);
//    }
//
//
//    public void setIntake2Power(double intake2Power){
//        intake2Right.setPower(intake2Power);
//        intake2Left.setPower(intake2Power);
//    }
//
//





}
