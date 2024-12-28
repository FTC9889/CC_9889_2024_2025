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
import com.qualcomm.robotcore.util.ElapsedTime;


public class Intake {
    public DcMotorEx extension;
    Servo intakeWristR, intakeWristL, intakeLock;
    CRServo intakeR, intakeL;
    DigitalChannel magnetSensor;
    ColorSensor colorSensor;

    private IntakeState CurrentIntakeState = IntakeState.NULL;
    private IntakeState RequestedIntakeState = IntakeState.NULL;

    private  WristState CurrentWristState = WristState.NULL;
    private  WristState RequstedWristState = WristState.NULL;

    private  PowerState CurrentPowerState = PowerState.NULL;
    private  PowerState RequstedPowerState = PowerState.NULL;

    private  SampleColor CurrentSampleColor = SampleColor.NULL;

    public IntakeState getCurrentIntakeState() {
        return CurrentIntakeState;
    }

    public WristState getCurrentWristState() {
        return CurrentWristState;
    }

    public PowerState getCurrentPowerState() {
        return CurrentPowerState;
    }

    public SampleColor getCurrentSampleColor() {
        return CurrentSampleColor;
    }

    public void setRequestedIntakeState(IntakeState requestedIntakeState) {
        RequestedIntakeState = requestedIntakeState;
    }

    public void setRequstedWristState(WristState requstedWristState) {
        RequstedWristState = requstedWristState;
    }

    public void setRequstedPowerState(PowerState requstedPowerState) {
        RequstedPowerState = requstedPowerState;
    }


    public enum IntakeState {
        INTAKE,
        RETRACTED,
        NULL,
        AUTO_EXTEND;

        public String toString() {
            switch (this) {
                case INTAKE:
                    return "INTAKE";
                case RETRACTED:
                    return "RETRACTED";
                case AUTO_EXTEND:
                    return "SLIGHT EXTEND";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }

    public enum WristState {
        DOWN_POSITION(0.1),
        MIDDLE_POSITION(0.5),
        UP_POSITION(1),
        NULL(0);

        private final double value;
        WristState(double value) {
            this.value = value;
        }

        public double getTargetPosition() {
            return value;
        }

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
        RED_YELLOW,
        BLUE_YELLOW,
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
                case RED_YELLOW:
                    return "RED_YELLOW";
                case BLUE_YELLOW:
                    return "BLUE_YELLOW";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }

    public enum PowerState{
        ON(1),
        OFF(0),
        SLOW(0.5),
        OUTTAKE(-1),
        NULL(0);

        private final double value;
        PowerState(double value) {
            this.value = value;
        }

        public double setTargetPower() {
            return value;
        }

        public String toString() {
            switch (this) {
                case ON:
                    return "ON";
                case OFF:
                    return "OFF";
                case SLOW:
                    return "SLOW";
                case OUTTAKE:
                    return "OUTTAKE";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }

    public void init(HardwareMap hardwareMap){
        extension = (DcMotorEx) hardwareMap.get(DcMotor.class,"extension");
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    final double closedPosition = 0;
    final double openPosition = 1;

    private void setExtensionLockPosition(double position){
        intakeLock.setPosition(position);
    }

    private final double extensionPower = 0;
    public void setExtensionPower(double power){
        double powerLevel = 0;
        if (magnetSensor.getState() && power < 0){
            powerLevel = 0;
        }else {
            powerLevel = power;
        }

        if (extensionPower != powerLevel)
            extension.setPower(powerLevel);
    }

    private void setIntakePower(double power){
        intakeL.setPower(power);
        intakeR.setPower(power);
    }

    private void setWristPosition(double position){
        intakeWristR.setPosition(position);
        intakeWristL.setPosition(position);
    }

    private class requestState implements Action {
        ElapsedTime wristTimer = new ElapsedTime();
        boolean resetWristTimer = false;

        ElapsedTime lockTimer = new ElapsedTime();
        ElapsedTime lockTimer2 = new ElapsedTime();
        ElapsedTime lockTimer3 = new ElapsedTime();

        int target = 0;

        public requestState(IntakeState intakeState, WristState wristState, PowerState powerState, SampleColor sampleColor) {
            this(intakeState, wristState, powerState, sampleColor, 0);
        }

        public requestState(IntakeState intakeState, WristState wristState, PowerState powerState, SampleColor sampleColor, int targetPosition) {
            setRequestedIntakeState(intakeState);
            setRequstedPowerState(powerState);

            if (intakeState != IntakeState.RETRACTED)
                setRequstedWristState(wristState);

            CurrentSampleColor = sampleColor;
            target = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

             // Intake Case
             if (RequestedIntakeState == IntakeState.INTAKE) {
                  setExtensionLockPosition(openPosition);

                  if (lockTimer.milliseconds() > 50)
                    CurrentIntakeState = RequestedIntakeState;
             } else {
                 lockTimer.reset();
             }

            // Retracted
            if (RequestedIntakeState == IntakeState.RETRACTED){
                if (magnetSensor.getState())
                    setExtensionLockPosition(closedPosition);
                else {
                    setExtensionLockPosition(openPosition);
                    lockTimer2.reset();
                }

                if (lockTimer2.milliseconds() > 50)
                    CurrentIntakeState = RequestedIntakeState;

                if (extension.getCurrentPosition() < 100){
                    extension.setPower(-0.3);
                }else {
                    extension.setPower(-1);
                }
            }

            // Auto Extend
            if (RequestedIntakeState == IntakeState.AUTO_EXTEND) {
                if (lockTimer3.milliseconds() < 50) {
                    setExtensionLockPosition(openPosition);
                } else {
                    extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extension.setTargetPosition(target);
                    extension.setPower(1);
                }

                if (Math.abs(extension.getCurrentPosition() - target) < 10) {
                    extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extension.setTargetPosition(target);
                    extension.setPower(0);
                    CurrentIntakeState = RequestedIntakeState;
                }
            } else {
                lockTimer3.reset();
            }

            // Wrist Control
             if (RequstedWristState != CurrentWristState) {
                 setWristPosition(RequstedWristState.getTargetPosition());

                 if (!resetWristTimer) wristTimer.reset();

                 if(wristTimer.seconds() >
                         0.1 * Math.abs(RequstedWristState.getTargetPosition() - CurrentWristState.getTargetPosition()))
                     CurrentWristState = RequstedWristState;

             } else {
                 resetWristTimer = false;
             }

             // Intake Power State
            if (RequstedPowerState != CurrentPowerState) {
                setIntakePower(RequstedPowerState.setTargetPower());
                CurrentPowerState = RequstedPowerState;
            }

            allowDriverExtension = CurrentIntakeState == IntakeState.INTAKE && RequestedIntakeState == IntakeState.INTAKE;

            return RequestedIntakeState == CurrentIntakeState || RequstedWristState == CurrentWristState || RequstedPowerState == CurrentPowerState;
        }
    }

    private boolean allowDriverExtension = false;
    public boolean allowDriverExtension(){
        return allowDriverExtension;
    }

    public Action Deployed() {
        return new requestState(IntakeState.INTAKE, WristState.DOWN_POSITION, PowerState.ON, CurrentSampleColor);
    }

    public Action Retracted() {
        return new requestState(IntakeState.RETRACTED, WristState.UP_POSITION, PowerState.SLOW, CurrentSampleColor);
    }

    public Action AutoSamples() {
        return new requestState(IntakeState.AUTO_EXTEND, WristState.DOWN_POSITION, PowerState.ON, CurrentSampleColor);
    }
}



