package com.team9889.ftc2024.subsystems;


import static com.team9889.ftc2024.subsystems.Lift.LiftState.HIGH_BASKET_POSITION;
import static com.team9889.ftc2024.subsystems.Lift.LiftState.HIGH_RUNG_POSITION;
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
// not going to requested state
@Config
public class Lift {
    public DcMotorEx liftMotorR, liftMotorL, liftMotor3;
    Servo elbowR, elbowL, wrist, claw, clutch;
//            shift1, shift2;
    public DigitalChannel liftMagnetSensor;

    public static double lift_kp = 0.025;
    public static double lift_kd = 0.02;
    double integralSum = 0;
    public static double lift_ki =  0.01;

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

    public enum TopLevelState {
        HIGH_BASKET(LiftState.HIGH_BASKET_POSITION, ElbowStates.BASKET_SCORE_POSITION, WristState.BASKET_SCORE_POSITION, ClawStates.CLOSED_POSITION),
        HIGH_BASKET_READY(LiftState.HIGH_BASKET_POSITION, ElbowStates.BASKET_SCORE_READY_POSITION, WristState.BASKET_SCORE_READY_POSITION, ClawStates.CLOSED_POSITION),
        HIGH_BASKET_RELEASE(LiftState.HIGH_BASKET_POSITION, ElbowStates.BASKET_SCORE_READY_POSITION, WristState.BASKET_SCORE_READY_POSITION, ClawStates.OPEN_POSITION),
        HIGH_RUNG(LiftState.HIGH_RUNG_POSITION, ElbowStates.RUNG_SCORE_POSITION, WristState.RUNG_SCORE_POSITION, ClawStates.CLOSED_POSITION),
        HIGH_RUNG_SCORED(LiftState.HIGH_RUNG_SCORE_POSITION, ElbowStates.RUNG_SCORE_POSITION, WristState.RUNG_SCORE_POSITION, ClawStates.CLOSED_POSITION),
        HIGH_RUNG_RELEASE(LiftState.HIGH_RUNG_RELEASED_POSITION, ElbowStates.RUNG_SCORE_POSITION, WristState.RUNG_SCORE_POSITION, ClawStates.OPEN_POSITION),
        TRANSFER_PREPARE(INTAKE_POSITION, ElbowStates.INTAKE_POSITION, WristState.INTAKE_POSITION, ClawStates.OPEN_POSITION),
        TRANSFER_READY(LiftState.TRANSFER_POSITION, ElbowStates.TRANSFER_POSITION, WristState.TRANSFER_POSITION, ClawStates.OPEN_POSITION),
        TRANSFER_COMPLETE(LiftState.TRANSFER_POSITION, ElbowStates.TRANSFER_POSITION, WristState.TRANSFER_POSITION, ClawStates.CLOSED_POSITION),
        SCORE_PREPARE(LiftState.DEFAULT_POSITION, ElbowStates.DEFAULT_POSITION, WristState.DEFAULT_POSITION, ClawStates.CLOSED_POSITION),
        HUMAN_PLAYER_POSITION(LiftState.HUMAN_PLAYER_POSITION, ElbowStates.HUMAN_PLAYER_POSITION, WristState.HUMAN_PLAYER_POSITION, ClawStates.OPEN_POSITION),
        HUMAN_PLAYER_GRABED(LiftState.HUMAN_PLAYER_POSITION, ElbowStates.HUMAN_PLAYER_POSITION,WristState.HUMAN_PLAYER_POSITION,ClawStates.CLOSED_POSITION),
        HUMAN_PLAYER_2(LiftState.HUMAN_PLAYER_POSITION_2,ElbowStates.HUMAN_PLAYER_POSITION_2,WristState.HUMAN_PLAYER_POSITION_2,ClawStates.CLOSED_POSITION),
        HANG_POSITION(LiftState.HUMAN_PLAYER_POSITION, ElbowStates.INTAKE_POSITION, WristState.INTAKE_POSITION, ClawStates.CLOSED_POSITION);

        final LiftState liftState;
        final ElbowStates elbowStates;
        final WristState wristState;
        final ClawStates clawStates;

        TopLevelState(LiftState liftstate, ElbowStates elbowStates, WristState wristState, ClawStates clawStates) {

            this.liftState = liftstate;
            this.elbowStates = elbowStates;
            this.wristState = wristState;
            this.clawStates = clawStates;
        }


    }

    public enum LiftState {
        NULL(0),
        INTAKE_POSITION(0),
        DEFAULT_POSITION(0),
        LOW_RUNG_POSITION(161),
        HIGH_RUNG_POSITION(647),
        HIGH_RUNG_RELEASED_POSITION(433),
        HIGH_RUNG_SCORE_POSITION(433),
        LOW_BASKET_POSITION(576),
        HIGH_BASKET_POSITION(1313),
        HUMAN_PLAYER_POSITION(41),
        HUMAN_PLAYER_POSITION_2(68),
        LEVEL_ONE_ASSENT_POSITION(445),
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
                case HUMAN_PLAYER_POSITION_2:
                    return "HUMAN_PLAYER_POSITION_2";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }


    public enum ElbowStates{
        NULL(0),
        INTAKE_POSITION(0.6),
        HUMAN_PLAYER_POSITION(0),
        RUNG_SCORE_POSITION(0.0),
        BASKET_SCORE_POSITION(0.2),
        BASKET_SCORE_READY_POSITION(0),
        HUMAN_PLAYER_POSITION_2(0.2),
        DEFAULT_POSITION(0.2),
        TRANSFER_POSITION(0.8);

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
                case HUMAN_PLAYER_POSITION_2:
                    return " HUMAN_PLAYER_POSITION_2";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }

    public enum WristState{
        NULL(0),
        INTAKE_POSITION(0),
        HUMAN_PLAYER_POSITION(0.85),
        HUMAN_PLAYER_POSITION_2(0.6),
        RUNG_SCORE_POSITION(0.85),
        BASKET_SCORE_POSITION(0.425),
        BASKET_SCORE_READY_POSITION(0.5),
        DEFAULT_POSITION(0.85),
        TRANSFER_POSITION(0.3),
        FIRST_SCORE(0.2);

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
                case HUMAN_PLAYER_POSITION_2:
                    return " HUMAN_PLAYER_POSITION_2";
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
        liftMotor3 = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotor3");

        elbowR = hardwareMap.servo.get("elbowr");
        elbowL = hardwareMap.servo.get("elbowl");

        wrist = hardwareMap.servo.get("wrist");
        claw = hardwareMap.servo.get("claw");

        clutch = hardwareMap.servo.get("clutch");

        liftMagnetSensor = hardwareMap.digitalChannel.get("liftMagnetSensor");

        liftMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        elbowL.setDirection(Servo.Direction.REVERSE);
    }

    public void setHangMotorPower(double power){
        liftMotor3.setPower(power);
    }

    public void setClutchPosition(double position) {
        clutch.setPosition(position);
    }

    double elbowPosition = 2;
    public void setElbowPosition(double position){
        if(position != elbowPosition) {
            elbowR.setPosition(position);
            elbowL.setPosition(position);
        }
        elbowPosition = position;
    }

    double wristPosition = 2;
    public void setWristPosition(double position){
            wrist.setPosition(position);
    }

    double clawPosition = 2;
    public void setClawPosition(double position){
        if(clawPosition != position)
            claw.setPosition(position);

        clawPosition = position;
    }

    double liftPower = 2;
    private void setMotorPower(double power){
        if (liftPower != power) {
            liftMotorL.setPower(power);
            liftMotorR.setPower(power);
        }
        liftPower = power;
    }

    public void setLiftMotorPower(double power){
        if (!liftMagnetSensor.getState()){
            if (power < 0) {
                setMotorPower(0);
            } else {
               setMotorPower(power);
            }

            offset = liftMotorR.getCurrentPosition();
        } else {
            setMotorPower(power);
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
        if (RequestedLiftState == HIGH_BASKET_POSITION) {
            return currentLiftPosition() > liftTargetPosition - 30;
        }
        return Math.abs(liftTargetPosition - currentLiftPosition()) <= 40;
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

    static ElapsedTime wristTimer = new ElapsedTime();
    static boolean resetWristTimer = true;

    static ElapsedTime elbowTimer = new ElapsedTime();
    static boolean resetElbowTimer = false;

    static ElapsedTime clawTimer = new ElapsedTime();
    static boolean resetClawTimer = false;

    public boolean takeover = false;

    /**
     * update = lift controller
     */
    public void update() {
        if (!takeover)
            setWristPosition(RequestedWristState.getTargetPosition());

        if (RequestedWristState != CurrentWristState) {

            if (resetWristTimer) {
                wristTimer.reset();
                resetWristTimer = false;
            }

            if(wristTimer.seconds() >
                    0.2 * Math.abs(RequestedWristState.getTargetPosition() - CurrentWristState.getTargetPosition()))
                CurrentWristState = RequestedWristState;

        } else {
            resetWristTimer = true;
        }

        if (RequestedElbowState != CurrentElbowState) {
            if(RequestedLiftState == HIGH_BASKET_POSITION && !liftInPosition()) {
                setElbowPosition(0.5);
            } else {
                setElbowPosition(RequestedElbowState.getTargetPosition());

                if (!resetElbowTimer) {
                    elbowTimer.reset();
                    resetElbowTimer = true;
                }

                if(elbowTimer.seconds() >
                        0.2 * Math.abs(RequestedElbowState.getTargetPosition() - CurrentElbowState.getTargetPosition()))
                    CurrentElbowState = RequestedElbowState;
            }
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
                    0.8 * Math.abs(RequestedClawState.getTargetPosition() - CurrentClawState.getTargetPosition()))
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

        if (RequestedLiftState == CurrentLiftState)
            lastGoodState = CurrentLiftState;

        double power;
        double error = liftTargetPosition - currentLiftPosition();
        if(liftTargetPosition != 0) {

            if (liftTimer != null && Math.abs(error) < 200)
                integralSum = integralSum + (error * liftTimer.seconds());

            power = lift_kp * error;

            if (RequestedLiftState == HIGH_BASKET_POSITION){
                power += lift_ki * integralSum;
            } else if (RequestedLiftState == HIGH_RUNG_POSITION) {
                power += (lift_ki/2) * integralSum;
            } else  {
                integralSum = 0;
            }

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
                power = -0.5;
            }

            integralSum = 0;

            setLiftMotorPower(power);
        }
    }

    public boolean isComplete() {
        return RequestedLiftState == CurrentLiftState && RequestedWristState == CurrentWristState && RequestedElbowState == CurrentElbowState && RequestedClawState == CurrentClawState;
    }

    double lastError = 0;
    ElapsedTime liftTimer = null;

    public void request(TopLevelState state){
        RequestedLiftState = state.liftState;
        RequestedElbowState = state.elbowStates;
        RequestedWristState = state.wristState;
        RequestedClawState = state.clawStates;
    }
}
