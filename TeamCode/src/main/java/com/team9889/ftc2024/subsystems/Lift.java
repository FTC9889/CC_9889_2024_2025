package com.team9889.ftc2024.subsystems;


import static com.team9889.ftc2024.subsystems.Lift.LiftState.INTAKE_POSITION;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Lift {
    DcMotorEx liftMotorR, liftMotorL, liftMotor3;
    Servo elbowR, elbowL, wrist, claw;
//            shift1, shift2;
    public DigitalChannel liftMagnetSensor;

    public static double lift_kp = 0.02;
    double lift_kd = 0.02;

    public LiftState CurrentLiftState = LiftState.NULL;
    public LiftState RequestedLiftState = LiftState.NULL;

    private ElbowStates CurrentElbowState = ElbowStates.NULL;
    public ElbowStates RequestedElbowState = ElbowStates.NULL;

    private WristState CurrentWristState = WristState.NULL;
    public WristState RequestedWristState = WristState.NULL;

    private ClawStates CurrentClawState = ClawStates.NULL;
    public ClawStates RequestedClawState = ClawStates.NULL;

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
        LOW_RUNG_POSITION(180),
        HIGH_RUNG_POSITION(648),
        HIGH_RUNG_RELEASED_POSITION(478),
        HIGH_RUNG_SCORE_POSITION(478),
        LOW_BASKET_POSITION(644),
        HIGH_BASKET_POSITION(1485 - 7),
        HUMAN_PLAYER_POSITION(0),
        LEVEL_ONE_ASSENT_POSITION(497),
        TRANSFER_POSITION(0);

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
                case  HIGH_RUNG_RELEASED_POSITION:
                    return " HIGH_RUNG_RELEASED_POSITION";
                case  HIGH_RUNG_SCORE_POSITION:
                    return " HIGH_RUNG_SCORE_POSITION";
                case  LOW_BASKET_POSITION:
                    return " LOW_BASKET_POSITION";
                case   HIGH_BASKET_POSITION:
                    return "  HIGH_BASKET_POSITION";
                case  HUMAN_PLAYER_POSITION:
                    return " HUMAN_PLAYER_POSITION";
                case TRANSFER_POSITION:
                    return "TRANSFER_POSITION";
                case LEVEL_ONE_ASSENT_POSITION:
                    return "LEVEL_ONE_ASSENT_POSITION";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }


    public enum ElbowStates{
        NULL(0),
        INTAKE_POSITION(0.6),
        HUMAN_PLAYER_POSITION(0.2),
        RUNG_SCORE_POSITION(0.2),
        BASKET_SCORE_POSITION(0.2),
        BASKET_SCORE_READY_POSITION(0),
        DEFAULT_POSITION(0.2),
        TRANSFER_POSITION(0.78);

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
                case  BASKET_SCORE_READY_POSITION:
                    return " BASKET_SCORE_READY_POSITION";
                case  DEFAULT_POSITION:
                    return " DEFAULT_POSITION";
                case   TRANSFER_POSITION:
                    return " TRANSFER_POSITION";

                case NULL:
                default:
                    return "NULL";
            }
        }
    }

    public enum WristState{
        NULL(0),
        INTAKE_POSITION(0.35),
        HUMAN_PLAYER_POSITION(0.6),
        RUNG_SCORE_POSITION(0.8),
        BASKET_SCORE_POSITION(0.8),
        BASKET_SCORE_READY_POSITION(0.6),
        DEFAULT_POSITION(0.8),
        TRANSFER_POSITION(0.43);

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
                case  BASKET_SCORE_READY_POSITION:
                    return " BASKET_SCORE_OUT_POSITION";
                case  DEFAULT_POSITION:
                    return " DEFAULT_POSITION";
                case   TRANSFER_POSITION:
                    return " TRANSFER_POSITION";

                case NULL:
                default:
                    return "NULL";
            }
        }
    }

    public enum ClawStates{
        CLOSED_POSITION(0.7),
        OPEN_POSITION(0.1),
        LOOSE_POSITION(0.6),
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
        liftMotorR = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotorR");
        liftMotorL = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotorL");
//        liftMotor3 = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftmotor3");

        elbowR = hardwareMap.servo.get("elbowr");
        elbowL = hardwareMap.servo.get("elbowl");

        wrist = hardwareMap.servo.get("wrist");
        claw = hardwareMap.servo.get("claw");

//        shift1 = hardwareMap.servo.get("shift1");
//        shift2 = hardwareMap.servo.get("shift2");

        liftMagnetSensor = hardwareMap.digitalChannel.get("liftMagnetSensor");

        liftMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

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

    public void setLiftMotorPower(double power){
        if (!liftMagnetSensor.getState()){
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

    private LiftState lastGoodState = INTAKE_POSITION;
    public LiftState lastLiftStateThatWasGood(){
        return lastGoodState;
    }

    public double getCurrentDraw() {
        return liftMotorR.getCurrent(CurrentUnit.MILLIAMPS) + liftMotorL.getCurrent(CurrentUnit.MILLIAMPS);
    }

    private int offset = 0;
    public int currentLiftPosition(){
        return -(liftMotorR.getCurrentPosition() - offset);
    }

    private boolean liftInPosition() {
        return Math.abs(liftTargetPosition - currentLiftPosition()) <= 20;
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
    public void setLiftPosition(int position){
        liftTargetPosition = position;
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

    static ElapsedTime wristTimer = new ElapsedTime();
    static boolean resetWristTimer = true;

    static ElapsedTime elbowTimer = new ElapsedTime();
    static boolean resetElbowTimer = false;

    static ElapsedTime clawTimer = new ElapsedTime();
    static boolean resetClawTimer = false;


    public class requestState implements Action {

        public requestState(LiftState liftState, ElbowStates elbowState, WristState wristState, ClawStates clawState) {
            setRequestedLiftState(liftState);
            setRequestedElbowState(elbowState);
            setRequestedWristState(wristState);
            setRequestedClawState(clawState);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            RobotLog.d("Timer: " + wristTimer.seconds() + " " + resetWristTimer);

            if (RequestedWristState != CurrentWristState) {
                setWristPosition(RequestedWristState.getTargetPosition());

                if (resetWristTimer) {
                    wristTimer.reset();
                    resetWristTimer = false;
                }

                if(wristTimer.seconds() >
                        0.1 * Math.abs(RequestedWristState.getTargetPosition() - CurrentWristState.getTargetPosition()))
                    CurrentWristState = RequestedWristState;

            } else {
                resetWristTimer = true;
            }


            if (RequestedElbowState != CurrentElbowState) {
                setElbowPosition(RequestedElbowState.getTargetPosition());

                if (!resetElbowTimer) {
                    elbowTimer.reset();
                    resetElbowTimer = true;
                }

                if(elbowTimer.seconds() >
                        0.1 * Math.abs(RequestedElbowState.getTargetPosition() - CurrentElbowState.getTargetPosition()))
                    CurrentElbowState = RequestedElbowState;

            } else {
                resetElbowTimer = false;
            }

            if (RequestedClawState != CurrentClawState) {
                setClawPosition(RequestedClawState.getTargetPosition());

                if (!resetClawTimer) {
                    clawTimer.reset();
                    resetClawTimer = true;
                }

                if(clawTimer.seconds() >
                        0.1 * Math.abs(RequestedClawState.getTargetPosition() - CurrentClawState.getTargetPosition()))
                    CurrentClawState = RequestedClawState;

            } else {
                resetClawTimer = false;
            }

            if(RequestedLiftState != CurrentLiftState) {
                setLiftPosition(RequestedLiftState.getTargetPosition());

                if(liftInPosition()) {
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
            if (RequestedLiftState == CurrentLiftState)
                lastGoodState = CurrentLiftState;

            double power;
            double error = liftTargetPosition - currentLiftPosition();
            if(liftTargetPosition != 0) {

                power = lift_kp * error;
                power = Math.min(power, 1);

                setLiftMotorPower(power);

                if(getCurrentDraw() > 10000 && power < -0.8) {
                    setRequestedLiftState(lastGoodState);
                    CurrentLiftState = INTAKE_POSITION;
                }

                // Reset Timer
                if (liftTimer == null) {
                    liftTimer = new ElapsedTime();
                } else {
                    liftTimer.reset();
                }

                // Set last error
                lastError = error;
            } else {
                if(currentLiftPosition() > 200) {
                    power = -1;
                } else {
                    power = -0.3;
                }

                setLiftMotorPower(power);
            }

            RobotLog.d("Power: " + power);

            return true;
        }
    }

    public Action LiftController(){
        return new LiftControl();
    }

    public Action HighRung() {
        return new requestState(LiftState.HIGH_RUNG_POSITION, ElbowStates.RUNG_SCORE_POSITION, WristState.RUNG_SCORE_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action HighRungScored(){
        return new requestState(LiftState.HIGH_RUNG_SCORE_POSITION, ElbowStates.RUNG_SCORE_POSITION, WristState.RUNG_SCORE_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action HighRungScoredReleased(){
        return new requestState(LiftState.HIGH_RUNG_RELEASED_POSITION, ElbowStates.RUNG_SCORE_POSITION, WristState.RUNG_SCORE_POSITION, ClawStates.OPEN_POSITION);
    }

    public Action LowRung(){
        return new requestState(LiftState.LOW_RUNG_POSITION, ElbowStates.RUNG_SCORE_POSITION, WristState.RUNG_SCORE_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action HighBasket() {
        return new requestState(LiftState.HIGH_BASKET_POSITION, ElbowStates.BASKET_SCORE_POSITION, WristState.BASKET_SCORE_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action HighBasketReady() {
        return new requestState(LiftState.HIGH_BASKET_POSITION, ElbowStates.BASKET_SCORE_READY_POSITION, WristState.BASKET_SCORE_READY_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action HighBasketReadyScored() {
        return new requestState(LiftState.HIGH_BASKET_POSITION, ElbowStates.BASKET_SCORE_READY_POSITION, WristState.BASKET_SCORE_READY_POSITION, ClawStates.OPEN_POSITION);
    }

    public Action LowBasket(){
        return new requestState(LiftState.LOW_BASKET_POSITION, ElbowStates.BASKET_SCORE_POSITION, WristState.BASKET_SCORE_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action HumanPlayer(){
        return new requestState(LiftState.HUMAN_PLAYER_POSITION, ElbowStates.HUMAN_PLAYER_POSITION, WristState.HUMAN_PLAYER_POSITION, ClawStates.OPEN_POSITION);
    }

    public Action HumanPlayerIntaked(){
        return new requestState(LiftState.HUMAN_PLAYER_POSITION, ElbowStates.HUMAN_PLAYER_POSITION, WristState.HUMAN_PLAYER_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action TransferPrepare(){
        return new requestState(INTAKE_POSITION, ElbowStates.INTAKE_POSITION, WristState.INTAKE_POSITION, ClawStates.OPEN_POSITION);
    }

    public Action TransferReady(){
        return new requestState(LiftState.TRANSFER_POSITION, ElbowStates.TRANSFER_POSITION, WristState.TRANSFER_POSITION, ClawStates.OPEN_POSITION);
    }

    public Action TransferComplete(){
        return new requestState(LiftState.TRANSFER_POSITION, ElbowStates.TRANSFER_POSITION, WristState.TRANSFER_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action ScorePrepare(){
        return new requestState(LiftState.DEFAULT_POSITION, ElbowStates.DEFAULT_POSITION, WristState.DEFAULT_POSITION, ClawStates.CLOSED_POSITION);
    }

    public Action AutoDrop(){
        return new requestState(LiftState.DEFAULT_POSITION, ElbowStates.DEFAULT_POSITION, WristState.BASKET_SCORE_POSITION, ClawStates.OPEN_POSITION);
    }
}
