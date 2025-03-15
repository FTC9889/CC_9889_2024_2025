package com.team9889.ftc2024.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Intake {

    double sampleLock = 0.83;
    double sampleUnlock = 0.34;

    public boolean auto = false;
    public boolean powerAllowed = true;
    public boolean disableColorsensor = false;


    public DcMotorEx extension;
    public Servo intakeWristR, intakeWristL, intakeLock, flicker;
    CRServo intakeR, intakeL;
    public DigitalChannel magnetSensor;
    public RevColorSensorV3 colorSensor;

    public IntakeState CurrentIntakeState = IntakeState.NULL;
    public IntakeState RequestedIntakeState = IntakeState.NULL;

    public  WristState CurrentWristState = WristState.NULL;
    public   WristState RequstedWristState = WristState.NULL;

    public  PowerState CurrentPowerState = PowerState.NULL;
    public   PowerState RequstedPowerState = PowerState.NULL;

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

    public void setIntakeLockPosition(double position){
        intakeLock.setPosition(position);
    }

    public void setIntakeWristPosition(double position){
        intakeWristL.setPosition(position);
        intakeWristR.setPosition(position);
    }

    public enum TopLevelState {
        RETRACTION(IntakeState.RETRACTED, WristState.UP_POSITION, PowerState.OFF, SampleColor.NULL),
        AUTO_RETRACTED(IntakeState.RETRACTED, WristState.UP_POSITION, PowerState.ON, SampleColor.NULL),
        AUTO_RETRACTED_2(IntakeState.AUTO_SPECIMEN_EXTEND, WristState.MIDDLE_POSITION, PowerState.ON, SampleColor.NULL, 200),

        OUTTAKE(IntakeState.RETRACTED, WristState.MIDDLE_POSITION, PowerState.OUTTAKE, SampleColor.NULL),
        TELEOP_OUTTAKE(IntakeState.INTAKE, WristState.MIDDLE_POSITION, PowerState.OUTTAKE, SampleColor.NULL),
        AUTO_OUTTAKE(IntakeState.RETRACTED, WristState.UP_POSITION, PowerState.OUTTAKE, SampleColor.NULL, 580),

        AUTO_SAMPLE(IntakeState.AUTO_EXTEND, WristState.DOWN_POSITION, PowerState.ON, SampleColor.NULL, 0),
        DEPLOY(IntakeState.INTAKE, WristState.DOWN_POSITION, PowerState.ON, SampleColor.NULL),
        DEPLOYED_OUTTAKE(IntakeState.INTAKE, WristState.DOWN_POSITION, PowerState.OUTTAKE, SampleColor.NULL),

        AUTO_SPECIMEN_1(IntakeState.AUTO_SPECIMEN_EXTEND, WristState.DOWN_POSITION, PowerState.ON, SampleColor.NULL, 240),
        AUTO_SPECIMEN_1_2(IntakeState.AUTO_SPECIMEN_EXTEND, WristState.DOWN_POSITION, PowerState.ON, SampleColor.NULL, 190),

        AUTO_SPECIMEN_2(IntakeState.AUTO_SPECIMEN_EXTEND, WristState.DOWN_POSITION, PowerState.ON, SampleColor.NULL, 550),
        AUTO_SPECIMEN_2_2(IntakeState.AUTO_SPECIMEN_EXTEND, WristState.DOWN_POSITION, PowerState.ON, SampleColor.NULL, 500),

        AUTO_SPECIMEN_3(IntakeState.AUTO_SPECIMEN_EXTEND, WristState.DOWN_POSITION, PowerState.ON, SampleColor.NULL, 580),
        AUTO_SPECIMEN_3_2(IntakeState.AUTO_SPECIMEN_EXTEND, WristState.DOWN_POSITION, PowerState.ON, SampleColor.NULL, 530),

        AUTO_PARK_EXTEND(IntakeState.AUTO_SPECIMEN_EXTEND, WristState.MIDDLE_POSITION, PowerState.OFF, SampleColor.NULL, 580);

        final IntakeState intakeState;
        final WristState wristState;
        final PowerState powerState;
        final SampleColor sampleColor;
        final int targetPosition;

        TopLevelState(IntakeState intakeState, WristState wristState, PowerState powerState, SampleColor sampleColor, int targetPosition) {
            this.intakeState = intakeState;
            this.wristState = wristState;
            this.powerState = powerState;
            this.sampleColor = sampleColor;
            this.targetPosition = targetPosition;

        }

        TopLevelState(IntakeState intakeState, WristState wristState, PowerState powerState, SampleColor sampleColor) {
            this(intakeState, wristState, powerState, sampleColor, 0);
        }
    }

    public enum IntakeState {
        INTAKE,
        RETRACTED,
        NULL,
        AUTO_EXTEND,
        AUTO_SPECIMEN_EXTEND;

        public String toString() {
            switch (this) {
                case INTAKE:
                    return "INTAKE";
                case RETRACTED:
                    return "RETRACTED";
                case AUTO_EXTEND:
                    return "AUTO_EXTEND";
                case AUTO_SPECIMEN_EXTEND:
                    return "AUTO_SPECIMEN_EXTEND";
                case NULL:
                default:
                    return "NULL";
            }
        }
    }

    public enum WristState {
        DOWN_POSITION(0.77),
        MIDDLE_POSITION(0.5),
        UP_POSITION(0.2),
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
        ON(-1),
        OFF(0),
        SLOW(-0.15),
        OUTTAKE(1),
        SLOWOUTTAKE(0.1),
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

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        flicker = hardwareMap.servo.get("flicker");

        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeWristL.setDirection(Servo.Direction.REVERSE);

        extension.setDirection(DcMotorSimple.Direction.REVERSE);

        powerAllowed = true;

        if (magnetSensor.getState() == false) {
            extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    final double closedPosition = 0.5;
    final double openPosition = 1;

    private void setExtensionLockPosition(double position){
        intakeLock.setPosition(position);
    }

    private double extensionPower = 0;
    public void setExtensionPower(double power){
        double powerLevel;
        if (!magnetSensor.getState() && power < 0){
            powerLevel = 0;

            extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }else {
            powerLevel = power;
        }

        if (extensionPower != powerLevel) {
            extension.setPower(powerLevel);
            extensionPower = powerLevel;
        }
    }

    public void setFlickerPosition(double position){
        flicker.setPosition(position);
    }

    double intakePower = 2;
    public void setIntakePower(double power){
        if (intakePower != power) {
            intakeL.setPower(power);
            intakeR.setPower(power);
        }
        intakePower = power;
        if(power == PowerState.OFF.value) {
            CurrentPowerState = PowerState.OFF;
        }
    }

    double wristPosition = 2;
    private void setWristPosition(double position){
        if(wristPosition != position) {
            intakeWristR.setPosition(position);
            intakeWristL.setPosition(position);
        }
        wristPosition = position;
    }



    public boolean sampleInIntake(){
        return colorSensor.getDistance(DistanceUnit.INCH) < 1.5;
    }

    public SampleColor getIntakeColor(){
        if (colorSensor.getDistance(DistanceUnit.INCH) < 1.5){
            double red = colorSensor.red();
            double green = colorSensor.green();
            double blue = colorSensor.blue();

            double total = red + green + blue;

            red /= total;
            green /= total;
            blue /= total;

            red *= 10;
            green *= 10;
            blue *= 10;

            if (red > blue && red > green) {
                return SampleColor.RED;
            } else if (blue > red && blue > green) {
                return SampleColor.BLUE;
            } else {
                return SampleColor.NEUTRAL;
            }
        } else {
            return SampleColor.NULL;
        }
    }


    private boolean allowDriverExtension = false;
    public boolean allowDriverExtension(){
        return allowDriverExtension;
    }


    ElapsedTime wristTimer = new ElapsedTime();
    boolean resetWristTimer = false;

    ElapsedTime lockTimer = new ElapsedTime();
    ElapsedTime lockTimer2 = new ElapsedTime();
    ElapsedTime lockTimer3 = new ElapsedTime();

    public int target = 0;

    public void update() {
        // Wrist Control
        if (RequstedWristState != CurrentWristState) {
            if(!resetWristTimer) {
                wristTimer.reset();
                resetWristTimer = true;
            }

            if(RequestedIntakeState == IntakeState.INTAKE) {
                if (extension.getCurrentPosition() < 100) {
                    setWristPosition(WristState.MIDDLE_POSITION.getTargetPosition());
                    CurrentWristState = WristState.MIDDLE_POSITION;
                } else {
                    setWristPosition(RequstedWristState.getTargetPosition());
                    if(wristTimer.milliseconds() > 500) {
                        CurrentWristState = RequstedWristState;
                    }
                }
            } else if (RequestedIntakeState == IntakeState.AUTO_EXTEND && CurrentIntakeState != IntakeState.AUTO_EXTEND) {
                if (extension.getCurrentPosition() < 100) {
                    setWristPosition(WristState.DOWN_POSITION.getTargetPosition());
                    CurrentWristState = WristState.DOWN_POSITION;
                } else {
                    setWristPosition(RequstedWristState.getTargetPosition()-0.03);
                    if(wristTimer.milliseconds() > 500) {
                        CurrentWristState = RequstedWristState;
                    }
                }
            } else {
                setWristPosition(RequstedWristState.getTargetPosition());

                if(wristTimer.milliseconds() > 500) {
                    CurrentWristState = RequstedWristState;
                }
            }
        } else {
            resetWristTimer = false;
        }

        // Intake Case
        if (RequestedIntakeState == IntakeState.INTAKE || RequestedIntakeState == IntakeState.AUTO_EXTEND || RequestedIntakeState == IntakeState.AUTO_SPECIMEN_EXTEND) {
            setExtensionLockPosition(openPosition);

            if (lockTimer.milliseconds() > 50)
                CurrentIntakeState = RequestedIntakeState;
        } else {
            lockTimer.reset();
        }

        // Retracted
        if (RequestedIntakeState == IntakeState.RETRACTED){
            if(extension.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
                extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (!magnetSensor.getState()) {
                setExtensionLockPosition(closedPosition);
                extension.setPower(0);

                if (auto){
                    setRequstedPowerState(PowerState.OFF);
                } else {
                    setRequstedPowerState(PowerState.ON);
                }

                extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                CurrentIntakeState = RequestedIntakeState;
            } else if (CurrentIntakeState != IntakeState.RETRACTED){
                    setExtensionLockPosition(openPosition);


                if (extension.getCurrentPosition() < 100){
                    extension.setPower(-0.7);
                    setRequstedWristState(WristState.UP_POSITION);
                    setRequstedPowerState(PowerState.ON);
                }else {
                    extension.setPower(-1);
                    setRequstedWristState(WristState.MIDDLE_POSITION);
                }
            }
        }

        // Auto Extend
        if (RequestedIntakeState == IntakeState.AUTO_EXTEND) {
            if (Math.abs(extension.getCurrentPosition() - target) < 40) {
                extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extension.setTargetPosition(target);
                extension.setPower(0);
                setExtensionLockPosition(closedPosition);

                CurrentIntakeState = RequestedIntakeState;
            } else {
                setExtensionLockPosition(openPosition);
                extension.setTargetPosition(target);
                extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extension.setPower(0.8);
            }

        }

        // Intake Power State
        if(powerAllowed) {
            setIntakePower(RequstedPowerState.setTargetPower());
        }
        CurrentPowerState = RequstedPowerState;

        allowDriverExtension = CurrentIntakeState == IntakeState.INTAKE && RequestedIntakeState == IntakeState.INTAKE;
    }

    public boolean isComplete() {
        return RequestedIntakeState == CurrentIntakeState && RequstedWristState == CurrentWristState & RequstedPowerState == CurrentPowerState;
    }

    public void requestState(TopLevelState state) {
        RequestedIntakeState = state.intakeState;
        RequstedWristState = state.wristState;
        RequstedPowerState = state.powerState;
        target = state.targetPosition;
    }
}



