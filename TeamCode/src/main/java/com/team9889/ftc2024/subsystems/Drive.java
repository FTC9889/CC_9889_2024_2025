package com.team9889.ftc2024.subsystems;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.team9889.lib.hardware.RevIMU;

public class Drive { DcMotor leftFront, rightFront;
//init hardware
public RevIMU imu;
//    private double lfPower;
//    private double rfPower;

    public void init(HardwareMap hardwareMap){
        leftFront=hardwareMap.dcMotor.get("leftfront");
        rightFront=hardwareMap.dcMotor.get("rightfront");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new RevIMU("imu", hardwareMap);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
    public void setPower(double RPower,double LPower){
        rightFront.setPower(LPower);
        leftFront.setPower(LPower);
        rightFront.setPower(RPower);
        leftFront.setPower(-RPower);

    }

    public void backward(double setPower){
        rightFront.setPower(setPower);
        leftFront.setPower(setPower);
    }

    public void forward(double setPower){
        rightFront.setPower(-setPower);
        leftFront.setPower(-setPower);
    }

    public void turnLeft(double setPower){
        rightFront.setPower(-setPower);
        leftFront.setPower(setPower);
    }

    public void turnRight(double setPower){
        rightFront.setPower(setPower);
        leftFront.setPower(-setPower);
    }



    public double get_angle(){
        return imu.getNormalHeading();
    }

    public int front_encoder(){
        return leftFront.getCurrentPosition();

    }
    public void reset_encoder(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void brake(){
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );

        setPower(0, 0);
    }
}

