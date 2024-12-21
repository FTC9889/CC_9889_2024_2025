package com.team9889.ftc2024.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.net.PortUnreachableException;


public class Intake {
    public DcMotorEx extension;
    Servo intakeWristR, intakeWristL, intakeLock;
    CRServo intakeR, intakeL;
    DigitalChannel magnetSensor;
    ColorSensor colorSensor;

    private IntakeState CurrentIntakeState = IntakeState.NULL;
    private IntakeState RequestedIntakeState = IntakeState.NULL;

    private WristState CurrentWristState = WristState.NULL;
    private WristState RequstedWristState = WristState.NULL;

    private PowerState CurrentPowerState = PowerState.NULL;
    private PowerState RequstedPowerState = PowerState.NULL;

    private SampleColor CurrentSampleColor = SampleColor.NULL;

    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        RETRACTED,
        NULL,
        SLIGHT_EXTEND;

        public String toString() {
            switch (this) {
                case INTAKE:
                    return "INTAKE";
                case OUTTAKE:
                    return "OUTTAKE";
                case RETRACTED:
                    return "RETRACTED";
                case SLIGHT_EXTEND:
                    return "SLIGHT EXTEND";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }

    public enum WristState {
        DOWN_POSITION,
        MIDDLE_POSITION,
        UP_POSITION,
        NULL;

        public String toString() {
            switch (this) {
                case DOWN_POSITION:
                    return "DOWN_POSITION";
                case MIDDLE_POSITION:
                    return "MIDDLE_POSITION";
                case UP_POSITION:
                    return "UP_POSITION";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }

    public enum SampleColor {
        NEUTRAL,
        RED,
        BLUE,
        NULL;

        public String toString() {
            switch (this) {
                case RED:
                    return "RED";
                case BLUE:
                    return "BLUE";
                case NEUTRAL:
                    return "NEUTRAL";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }

    public enum PowerState{
        ON,
        OFF,
        NULL;

        public String toString() {
            switch (this) {
                case ON:
                    return "ON";
                case OFF:
                    return "OFF";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }

    public void init(HardwareMap hardwareMap){
        extension = (DcMotorEx) hardwareMap.get(DcMotor.class,"extension");

        intakeWristR = hardwareMap.servo.get("intakeWristR");
        intakeWristL = hardwareMap.servo.get("intakeWristL");
        intakeLock = hardwareMap.servo.get("intakeLock");

        intakeR = hardwareMap.crservo.get("intakeR");
        intakeL = hardwareMap.crservo.get("intakeL");

        magnetSensor = hardwareMap.digitalChannel.get("magnetSensor");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeWristL.setDirection(Servo.Direction.REVERSE);


    }

    public void setWristPosision(double position){
        intakeWristR.setPosition(position);
        intakeWristL.setPosition(position);
    }

    public class wristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setWristPosision(0.5);
            return false;
        }
    }

    public Action wristDown(){
        return new wristDown();
    }
}

