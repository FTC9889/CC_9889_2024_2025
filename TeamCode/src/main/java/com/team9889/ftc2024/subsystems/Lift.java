package com.team9889.ftc2024.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {
    DcMotorEx liftMotorR, liftMotorL, liftMotor3;
    Servo elbowR, elbowL, wrist, claw, shift1, shift2;
    DigitalChannel magnetSensor;

    final double closedClaw = 0;
    final double openClaw = 1;
    final double looseClaw = 0.5;

    double lift_kp = 0.1;
    double lift_kd = 0.01;

    private LiftState CurrentLiftState = LiftState.NULL;
    private LiftState RequestedLiftState = LiftState.NULL;

    private ElbowStates CurrentElbowState = ElbowStates.NULL;
    private ElbowStates RequestedElbowState = ElbowStates.NULL;

    private WristState CurrentWristState = WristState.NULL;
    private WristState RequestedWristState = WristState.NULL;

    private ClawStates CurrentClawState = ClawStates.NULL;
    private ClawStates RequestedClawState = ClawStates.NULL;

    public LiftState getCurrentLiftState() {
        return CurrentLiftState;
    }

    public ElbowStates getCurrentElbowState() {
        return CurrentElbowState;
    }
    public WristState getCurrentWristState() {
        return CurrentWristState;
    }
    public ClawStates getCurrentClawState() {
        return CurrentClawState;
    }
    public void setRequestedClawState(ClawStates requestedClawState) {
        RequestedClawState = requestedClawState;
    }

    public void setRequestedLiftState(LiftState requestedLiftState) {
        RequestedLiftState = requestedLiftState;
    }

    public void setRequestedElbowState(ElbowStates requestedElbowState) {
        RequestedElbowState = requestedElbowState;
    }

    public void setRequestedWristState(WristState requestedWRistState) {
        RequestedWristState = requestedWRistState;
    }

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

        private final int value;
        LiftState(int value) {
            this.value = value;
        }

        public int getTargetPosition() {
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

    public void setElbowPosition(double position){
        elbowR.setPosition(position);
        elbowL.setPosition(position);
    }
    public void setWristPosition(double position){
        wrist.setPosition(position);
    }
    public void setClawPosition(double position){
        claw.setPosition(position);
    }

    private double lastPower = 1;
    public void setLiftMotorPower(double power){
        if (lastPower != power) {
            if (magnetSensor.getState() == true){
                if (power < 0) {
                    liftMotorR.setPower(0);
                    liftMotorL.setPower(0);
                } else {
                    liftMotorL.setPower(power);
                    liftMotorR.setPower(power);
                }

                offset = liftMotorR.getCurrentPosition();
            } else {
                liftMotorL.setPower(power);
                liftMotorR.setPower(power);
            }
        }

        lastPower = power;
    }

    private int offset = 0;
    private int currentLiftPosition(){
        return liftMotorR.getCurrentPosition() - offset;
    }

    private boolean liftInPosition = false;
    private boolean liftInPosition() {
        return liftInPosition;
    }

    private boolean wristInPosition = false;
    private boolean wristInPosition() {
        return wristInPosition;
    }

    private boolean elbowInPosition= false;
    private boolean elbowInPosition() {
        return elbowInPosition;
    }

    private boolean clawInPosition = false;
    private boolean clawInPosition() {
        return clawInPosition;
    }

    private int liftTargetPosition = 0;
    private int lastTarget = 0;
    public void setLiftPosition(int position){
        liftTargetPosition = position;
        if(liftTargetPosition != lastTarget) liftInPosition = false;
        lastTarget = liftTargetPosition;
    }

    public void closedClaw(){
        claw.setPosition(ClawStates.CLOSED_POSITION.getTargetPosition());
    }

    public void openClaw(){
        claw.setPosition(ClawStates.OPEN_POSITION.getTargetPosition());
    }

    public void looseClaw(){
        claw.setPosition(ClawStates.LOOSE_POSITION.getTargetPosition());
    }

    public class openClaw implements Action {
        ElapsedTime timer = null;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null)
                timer = new ElapsedTime();

            openClaw();

            return timer.milliseconds() < 50;
        }
    }

    public Action ReleaseClaw() {
        return new openClaw();
    }



    public class requestState implements Action {
        public requestState(LiftState liftState, ElbowStates elbowState, WristState wristState, ClawStates clawState) {
            setRequestedLiftState(liftState);
            setRequestedElbowState(elbowState);
            setRequestedWristState(wristState);
            setRequestedClawState(clawState);
        }

        ElapsedTime wristTimer = new ElapsedTime();
        boolean resetWristTimer = false;

        ElapsedTime elbowTimer = new ElapsedTime();
        boolean resetElbowTimer = false;

        ElapsedTime clawTimer = new ElapsedTime();
        boolean resetClawTimer = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (RequestedWristState != CurrentWristState) {
                setWristPosition(RequestedWristState.getTargetPosition());

                if (!resetWristTimer) wristTimer.reset();

                if(wristTimer.seconds() >
                        0.1 * Math.abs(RequestedWristState.getTargetPosition() - CurrentWristState.getTargetPosition()))
                    CurrentWristState = RequestedWristState;

            } else {
                resetWristTimer = false;
            }

            if (RequestedElbowState != CurrentElbowState) {
                setElbowPosition(RequestedElbowState.getTargetPosition());

                if (!resetElbowTimer) elbowTimer.reset();

                if(elbowTimer.seconds() >
                        0.1 * Math.abs(RequestedElbowState.getTargetPosition() - CurrentElbowState.getTargetPosition()))
                    CurrentElbowState = RequestedElbowState;

            } else {
                resetElbowTimer = false;
            }

            if (RequestedClawState != CurrentClawState) {
                setClawPosition(RequestedClawState.getTargetPosition());

                if (!resetClawTimer) clawTimer.reset();

                if(clawTimer.seconds() >
                        0.1 * Math.abs(RequestedClawState.getTargetPosition() - CurrentClawState.getTargetPosition()))
                    CurrentClawState = RequestedClawState;

            } else {
                resetClawTimer = false;
            }

            if(RequestedLiftState != CurrentLiftState) {
                setLiftPosition(CurrentLiftState.getTargetPosition());
                if(liftInPosition) {
                    CurrentLiftState = RequestedLiftState;
                }
            }

            return RequestedLiftState != CurrentLiftState || RequestedWristState != CurrentWristState || RequestedElbowState != CurrentElbowState || RequestedClawState != CurrentClawState;
        }
    }

    double lastError = 0;
    ElapsedTime liftTimer = null;
    public class LiftControl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double error = liftTargetPosition - currentLiftPosition();
            double derror = 0;

            if(liftTimer != null) derror = (error - lastError) / liftTimer.seconds();

            double power = lift_kp * error + lift_kd * derror;
            power = Math.max(power, 1);

            setLiftMotorPower(power);

            liftInPosition = Math.abs(error) <= 20;

            // Reset Timer
            if(liftTimer == null) {
                liftTimer = new ElapsedTime();
            } else {
                liftTimer.reset();
            }

            // Set last error
            lastError = error;

            return true;
        }
    }

    public Action LiftController(){
        return new LiftControl();
    }

    public Action HighRung() {
        return new requestState(LiftState.HIGH_RUNG_POSITION, ElbowStates.RUNG_SCORE_POSITION, WristState.RUNG_SCORE_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action LowRung(){
        return new requestState(LiftState.LOW_RUNG_POSITION, ElbowStates.RUNG_SCORE_POSITION, WristState.RUNG_SCORE_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action HighBasket() {
        return new requestState(LiftState.HIGH_BASKET_POSITION, ElbowStates.BASKET_SCORE_POSITION, WristState.BASKET_SCORE_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action LowBasket(){
        return new requestState(LiftState.LOW_BASKET_POSITION, ElbowStates.BASKET_SCORE_POSITION, WristState.BASKET_SCORE_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action HumanPlayer(){
        return new requestState(LiftState.HUMAN_PLAYER_POSITION, ElbowStates.HUMAN_PLAYER_POSITION, WristState.HUMAN_PLAYER_POSITION, ClawStates.OPEN_POSITION);
    }

    public Action TransferPrepare(){
        return new requestState(LiftState.INTAKE_POSITION, ElbowStates.INTAKE_POSITION, WristState.INTAKE_POSITION, ClawStates.OPEN_POSITION);
    }

    public Action ScorePrepare(){
        return new requestState(LiftState.DEFAULT_POSITION, ElbowStates.DEFAULT_POSITION, WristState.DEFAULT_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action HangUp(){
        return new requestState(LiftState.HANG_DEPLOYED_POSITION, ElbowStates.DEFAULT_POSITION, WristState.DEFAULT_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action HangDown(){
        return new requestState(LiftState.HANG_RETRACTED_POSITION, ElbowStates.DEFAULT_POSITION, WristState.DEFAULT_POSITION, ClawStates.CLOSED_POSITION);
    }









}
