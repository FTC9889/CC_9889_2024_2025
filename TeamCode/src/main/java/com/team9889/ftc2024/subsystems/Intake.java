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
        ON,
        OFF,
        SLOW,
        OUTTAKE,
        NULL;

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

    public void setExtensionPower(double power){
        extension.setPower(power);
    }

    public void setIntakePower(double power){
        intakeL.setPower(power);
        intakeR.setPower(power);
    }


    public void intake(){
        intakeR.setPower(1);
        intakeL.setPower(1);
    }

    public void outtake(){
        intakeR.setPower(-1);
        intakeL.setPower(-1);
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
    private boolean intakeInPosition = false;
    private boolean intakeInPosition() {
        return intakeInPosition;
    }




    public class requestState implements Action {
        public requestState(IntakeState intakeState, WristState wristState, PowerState powerState,SampleColor sampleColor ) {
            setRequestedIntakeState(intakeState);
            setRequstedPowerState(powerState);

            if (intakeState != IntakeState.RETRACTED)
                setRequstedWristState(wristState);

            CurrentSampleColor = sampleColor;
        }

        ElapsedTime wristTimer = new ElapsedTime();
        boolean resetWristTimer = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

             switch (RequestedIntakeState) {
                 case INTAKE:
                     CurrentIntakeState = RequestedIntakeState;
                     break;
                 case RETRACTED:
                     switch (CurrentIntakeState){
                         case INTAKE:
                             if(CurrentWristState == WristState.MIDDLE_POSITION) {

                             } else {
                                 setRequstedWristState(WristState.MIDDLE_POSITION);
                             }
                         case RETRACTED:
                             break;
                         case NULL:
                             break;
                         case AUTO_EXTEND:
                             break;
                     }
             }

             if (RequstedWristState != CurrentWristState) {
                 switch (RequstedWristState) {
                     case DOWN_POSITION:
                         setWristPosision(WristState.DOWN_POSITION.getTargetPosition());
                         break;
                     case UP_POSITION:
                         setWristPosision(WristState.UP_POSITION.getTargetPosition());
                         break;
                     case MIDDLE_POSITION:
                         setWristPosision(WristState.MIDDLE_POSITION.getTargetPosition());
                         break;
                     case NULL:
                     default:
                         break;
                 }

                 if (!resetWristTimer) wristTimer.reset();

                 if(wristTimer.seconds() >
                         0.1 * Math.abs(RequstedWristState.getTargetPosition() - CurrentWristState.getTargetPosition()))
                     CurrentWristState = RequstedWristState;

             } else {
                 resetWristTimer = false;
             }

            if(CurrentIntakeState == IntakeState.INTAKE) {
                setExtensionPower(driverIntakeCommand);
            }

            if (RequstedPowerState != CurrentPowerState) {
                switch (RequstedPowerState) {
                    case ON:
                        setIntakePower(1);
                        break;
                    case OFF:
                        setIntakePower(0);
                        break;
                    case OUTTAKE:
                        setIntakePower(-1);
                        break;
                    case SLOW:
                        setIntakePower(0.5);
                        break;
                    case NULL:
                    default:
                        break;
                }

                CurrentPowerState = RequstedPowerState;
            }



            return false;
        }
    }

    double driverIntakeCommand = 0;

    public void setDriverIntakeCommand(double joystick){
        driverIntakeCommand = joystick;
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



