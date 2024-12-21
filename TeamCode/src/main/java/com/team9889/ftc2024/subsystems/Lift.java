package com.team9889.ftc2024.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.security.PublicKey;

public class Lift {
    public
    DcMotorEx liftMotorR, liftMotorL, liftMotor3;
    Servo elbowR, elbowL, wrist, claw, shift1, shift2;
    DigitalChannel magnetSensor;

    private LiftState CurrentLiftState = LiftState.NULL;
    private LiftState RequestedLiftState = LiftState.NULL;

    private ElbowStates CurrentElbowState = ElbowStates.NULL;
    private ElbowStates RequestedElbowState = ElbowStates.NULL;

    private WristState CurrentWristState = WristState.NULL;
    private WristState RequestedWRistState = WristState.NULL;

    private ClawStates CurrentClawState = ClawStates.NULL;
    private ClawStates RequestedClawState = ClawStates.NULL;

    public enum LiftState {
        NULL,
        INTAKE_POSITION,
        DEFAULT_POSITION,
        LOW_RUNG_POSITION,
        HIGH_RUNG_POSITION,
        LOW_BASKET_POSITION,
        HIGH_BASKET_POSITION,
        HUMAN_PLAYER_POSITION,
        HANG_RETRACTED_POSITION,
        HANG_DEPLOYED_POSITION;

        public String toString() {
            switch (this) {
                case INTAKE_POSITION:
                    return "Intake Position";
                case DEFAULT_POSITION:
                    return "DEFAULT_POSITION";
                case LOW_RUNG_POSITION:
                    return "LOW_RUNG_POSITION";
                case  HIGH_RUNG_POSITION:
                    return " HIGH_RUNG_POSITION";
                case  LOW_BASKET_POSITION:
                    return " LOW_BASKET_POSITION";
                case   HIGH_BASKET_POSITION:
                    return "  HIGH_BASKET_POSITION";
                case  HUMAN_PLAYER_POSITION:
                    return " HUMAN_PLAYER_POSITION";
                case  HANG_RETRACTED_POSITION:
                    return " HANG_RETRACTED_POSITION";
                case  HANG_DEPLOYED_POSITION:
                    return " HANG_DEPLOYED_POSITION";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }


    public enum ElbowStates{
        NULL,
        INTAKE_POSITION,
        HUMAN_PLAYER_POSITION,
        RUNG_SCORE_POSITION,
        BASKET_SCORE_POSITION,
        DEFAULT_POSITION;

        public String toString() {
            switch (this) {
                case   INTAKE_POSITION:
                    return "  INTAKE_POSITION";
                case  HUMAN_PLAYER_POSITION:
                    return " HUMAN_PLAYER_POSITION";
                case  RUNG_SCORE_POSITION:
                    return " RUNG_SCORE_POSITION";
                case  BASKET_SCORE_POSITION:
                    return " BASKET_SCORE_POSITION";
                case  DEFAULT_POSITION:
                    return " DEFAULT_POSITION";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }

    public enum WristState{
        NULL,
        INTAKE_POSITION,
        HUMAN_PLAYER_POSITION,
        RUNG_SCORE_POSITION,
        BASKET_SCORE_POSITION,
        DEFAULT_POSITION;
        public String toString() {
            switch (this) {
                case   INTAKE_POSITION:
                    return "  INTAKE_POSITION";
                case  HUMAN_PLAYER_POSITION:
                    return " HUMAN_PLAYER_POSITION";
                case  RUNG_SCORE_POSITION:
                    return " RUNG_SCORE_POSITION";
                case  BASKET_SCORE_POSITION:
                    return " BASKET_SCORE_POSITION";
                case  DEFAULT_POSITION:
                    return " DEFAULT_POSITION";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }

    public enum ClawStates{
        CLOSED_POSITION,
        OPEN_POSITION,
        LOOSE_POSITION,
        NULL;

        public String toString() {
            switch (this) {
                case   CLOSED_POSITION:
                    return "  CLOSED_POSITION";
                case  OPEN_POSITION:
                    return " OPEN_POSITION";
                case  LOOSE_POSITION:
                    return " LOOSE_POSITION";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }

    public void init(HardwareMap hardwareMap){
        liftMotorR = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftmotorR");
        liftMotorL = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftmotorL");
        liftMotor3 = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftmotor3");

        elbowR = hardwareMap.servo.get("elbowr");
        elbowL = hardwareMap.servo.get("elbowl");
        wrist = hardwareMap.servo.get("wrist");
        claw = hardwareMap.servo.get("claw");

        shift1 = hardwareMap.servo.get("shift1");
        shift2 = hardwareMap.servo.get("shift2");

        magnetSensor = hardwareMap.digitalChannel.get("magnetSensor");

        liftMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        elbowL.setDirection(Servo.Direction.REVERSE);
    }


}
