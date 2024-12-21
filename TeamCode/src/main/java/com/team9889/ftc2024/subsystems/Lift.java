package com.team9889.ftc2024.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    public
    DcMotorEx liftMotorR, liftMotorL, liftMotor3;
    Servo elbowR, elbowL, wrist, claw, shift1, shift2;
    DigitalChannel magnetSensor;

    private final LiftState CurrentLiftState = LiftState.NULL;
    private final LiftState RequestedLiftState = LiftState.NULL;

    private final ElbowStates CurrentElbowState = ElbowStates.NULL;
    private final ElbowStates RequestedElbowState = ElbowStates.NULL;

    private final WristState CurrentWristState = WristState.NULL;
    private final WristState RequestedWRistState = WristState.NULL;

    private final ClawStates CurrentClawState = ClawStates.NULL;
    private final ClawStates RequestedClawState = ClawStates.NULL;

    public enum LiftState {
        NULL(0),
        INTAKE_POSITION(0),
        DEFAULT_POSITION(0),
        LOW_RUNG_POSITION(200),
        HIGH_RUNG_POSITION(700),
        LOW_BASKET_POSITION(400),
        HIGH_BASKET_POSITION(800),
        HUMAN_PLAYER_POSITION(100),
        HANG_RETRACTED_POSITION(0),
        HANG_DEPLOYED_POSITION(2000);

        private final double value;
        LiftState(double value) {
            this.value = value;
        }

        public double getTargetPosition() {
            return value;
        }

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
        NULL(0),
        INTAKE_POSITION(0.1),
        HUMAN_PLAYER_POSITION(0.2),
        RUNG_SCORE_POSITION(0.3),
        BASKET_SCORE_POSITION(0.4),
        DEFAULT_POSITION(0.5);

        private final double value;
        ElbowStates(double value) {
            this.value = value;
        }

        public double getTargetPosition() {
            return value;
        }

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
        NULL(0),
        INTAKE_POSITION(0.1),
        HUMAN_PLAYER_POSITION(0.2),
        RUNG_SCORE_POSITION(0.3),
        BASKET_SCORE_POSITION(0.4),
        DEFAULT_POSITION(0.5);

        private final double value;
        WristState(double value) {
            this.value = value;
        }

        public double getTargetPosition() {
            return value;
        }

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
        CLOSED_POSITION(0.1),
        OPEN_POSITION(0.2),
        LOOSE_POSITION(0.3),
        NULL(0);

        private final double value;
        ClawStates(double value) {
            this.value = value;
        }

        public double getTargetPosition() {
            return value;
        }

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
