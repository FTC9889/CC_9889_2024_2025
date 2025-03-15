package com.team9889.ftc2024.opmode;
import android.accounts.AccountAuthenticatorResponse;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2024.subsystems.Intake;
import com.team9889.ftc2024.subsystems.Lift;
import com.team9889.ftc2024.subsystems.Robot;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TeleOperate extends OpMode {
    Robot mRobot = new Robot();
    boolean score = false;
    boolean sampleInRobot = false;
    boolean something = false;
    boolean yellow = true;
    String color = "Nothing";
    boolean clutch = false;

    double x = 0;
    double y = 0;

    double unlocked = 0.8;
    double locked = 0;

    double sampleLock = 0.83;
    double sampleUnlock = 0.34;

    ElapsedTime timer = new ElapsedTime();
    boolean somethingElse = false;
    boolean safe = false;
    boolean press = false;


    Pose holdPoint = new Pose(0,0,0);
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime lockTimer = new ElapsedTime();

    Intake.SampleColor allianceColor = Intake.SampleColor.RED;
    Intake.SampleColor opponentColor = Intake.SampleColor.BLUE;

    @Override
    public void init() {
        mRobot.init(hardwareMap);
        mRobot.mIntake.auto = true;

        mRobot.mDrive.setStartingPose(Robot.robotPose);
    }

    @Override
    public void start() {
        mRobot.mDrive.startTeleopDrive();
    }

    @Override
    public void loop() {

        if (gamepad2.back && gamepad2.x){
            allianceColor = Intake.SampleColor.BLUE;
            opponentColor = Intake.SampleColor.RED;
        }
        if (gamepad2.back && gamepad2.b){
            allianceColor = Intake.SampleColor.RED;
            opponentColor = Intake.SampleColor.BLUE;
        }

        // Set Auto reject Neutral Samples
        if (gamepad2.dpad_up){
            yellow = false;
        } else if (gamepad2.dpad_down) {
            yellow = true;
        }

        // Moved safe distance away from High Basket
        safe = Math.abs(Math.hypot(mRobot.mDrive.getPose().getX() - x, mRobot.mDrive.getPose().getY() - y)) > 10;

        // Driver Control over drivetrain
        if(Math.abs(-gamepad1.left_stick_y) > 0.01 ||  Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.right_stick_x) > 0.01) {
            mRobot.mDrive.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            holdPoint = mRobot.mDrive.getPose();
        }
        else {
            mRobot.mDrive.setTeleOpMovementVectors(0, 0, 0);
        }

        // Controls the Intake Extension
        if (mRobot.mIntake.allowDriverExtension() || gamepad1.dpad_left){
            if(Math.abs(gamepad1.right_trigger) > Math.abs(gamepad1.left_trigger))
                mRobot.mIntake.setExtensionPower(gamepad1.right_trigger);
            else if(Math.abs(gamepad1.right_trigger) < Math.abs(gamepad1.left_trigger))
                mRobot.mIntake.setExtensionPower(-gamepad1.left_trigger);
            else
                mRobot.mIntake.setExtensionPower(0);
        }


        // Hang Controls
        mRobot.mLift.liftMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mRobot.mLift.setHangMotorPower(-gamepad2.left_stick_y);

        // Flag Controls
        if (gamepad1.dpad_up){ // Endgame Hack
            mRobot.mFlag.setFlagPosition(0.55);
        }else { // Normal Flag Control (Show auto reject)
            if (!yellow) {
                mRobot.mFlag.setFlagPosition(0.85);
            } else {
                mRobot.mFlag.setFlagPosition(0.9);
            }
        }

        // Check if we are in intaking
        if (mRobot.mIntake.getCurrentIntakeState() == Intake.IntakeState.INTAKE) {

            // If we detect the correct sample:
            if (mRobot.mIntake.getIntakeColor() == allianceColor ||
                    (mRobot.mIntake.getIntakeColor() == Intake.SampleColor.NEUTRAL && yellow)) {
                if (lockTimer.milliseconds() > 500) {
                    mRobot.mIntake.setFlickerPosition(sampleLock);
                }

                if (mRobot.mIntake.magnetSensor.getState()){
                    mRobot.mIntake.requestState(Intake.TopLevelState.RETRACTION);
                    sampleInRobot = true;
                    score = false;
                }
            } else {
                lockTimer.reset();
            }

            // If we detect the wrong sample:
            if (((mRobot.mIntake.getIntakeColor() == Intake.SampleColor.NEUTRAL && !yellow) || mRobot.mIntake.getIntakeColor()  == opponentColor
                    && mRobot.mIntake.getCurrentWristState() == Intake.WristState.DOWN_POSITION) || gamepad1.y){
                mRobot.mIntake.requestState(Intake.TopLevelState.TELEOP_OUTTAKE);
                mRobot.mIntake.setFlickerPosition(sampleUnlock);
            }
        }


        if (!score) {

            // Automatic Transfer
            if (mRobot.mIntake.getCurrentWristState() == Intake.WristState.UP_POSITION
                    && mRobot.mIntake.getCurrentIntakeState() == Intake.IntakeState.RETRACTED
                    && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.INTAKE_POSITION) {
                mRobot.mLift.request(Lift.TopLevelState.TRANSFER_READY);
            } else if (mRobot.mIntake.getCurrentIntakeState() == Intake.IntakeState.RETRACTED
                    && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.TRANSFER_POSITION
                    && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.OPEN_POSITION) {
                mRobot.mLift.request(Lift.TopLevelState.TRANSFER_COMPLETE);
                mRobot.mIntake.setFlickerPosition(sampleUnlock);
                lockTimer.reset();
                mRobot.mIntake.setIntakePower(Intake.PowerState.OFF.setTargetPower());
            } else if (mRobot.mIntake.getCurrentIntakeState() == Intake.IntakeState.RETRACTED
                    && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION
                    && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.TRANSFER_POSITION
                    && mRobot.mLift.getCurrentLiftState() == Lift.LiftState.TRANSFER_POSITION) {
                mRobot.mLift.takeover = true;
                mRobot.mLift.setWristPosition(Lift.WristState.FIRST_SCORE.getTargetPosition());
                mRobot.mLift.request(Lift.TopLevelState.SCORE_PREPARE);
                timer.reset();
            } else if(mRobot.mLift.getCurrentLiftState() == Lift.LiftState.DEFAULT_POSITION && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.DEFAULT_POSITION && mRobot.mLift.getCurrentWristState() == Lift.WristState.DEFAULT_POSITION && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION && timer.milliseconds() > 500) {
                mRobot.mLift.setWristPosition(Lift.WristState.DEFAULT_POSITION.getTargetPosition());
                mRobot.mLift.takeover = false;
            }

            // If the claw is closed (sample in robot), go to high basket
            if (gamepad2.a && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION
                    && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.DEFAULT_POSITION) {
                mRobot.mLift.request(Lift.TopLevelState.HIGH_BASKET_READY);
                mRobot.mIntake.requestState(Intake.TopLevelState.OUTTAKE);
            }

            // If in the high basket state, go to score
            if (gamepad1.right_bumper && mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HIGH_BASKET_POSITION) {
                mRobot.mLift.request(Lift.TopLevelState.HIGH_BASKET_RELEASE);
                mRobot.mIntake.requestState(Intake.TopLevelState.RETRACTION);
                sampleInRobot = false;
                x = mRobot.mDrive.getPose().getX();
                y = mRobot.mDrive.getPose().getY();
            }

            // If in high basket turn off
            if (mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HIGH_BASKET_POSITION && mRobot.mIntake.getCurrentIntakeState() == Intake.IntakeState.RETRACTED)
                mRobot.mIntake.setIntakePower(0);
        }

        if(mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HIGH_BASKET_POSITION
                && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.BASKET_SCORE_READY_POSITION
                && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.OPEN_POSITION
                && safe) {
            mRobot.mLift.takeover = false;
            mRobot.mLift.request(Lift.TopLevelState.TRANSFER_PREPARE);
            mRobot.mIntake.requestState(Intake.TopLevelState.RETRACTION);
            score = true;
            mRobot.mIntake.auto = true;
        }

        // Allow human player alignment in these states
        boolean inIntaking = mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.INTAKE_POSITION && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.OPEN_POSITION;
        boolean inSomething = mRobot.mLift.getCurrentLiftState() == Lift.LiftState.DEFAULT_POSITION
                && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.DEFAULT_POSITION
                && mRobot.mLift.getCurrentWristState() == Lift.WristState.DEFAULT_POSITION
                && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION;

        boolean inStartOfTeleop = mRobot.mLift.getCurrentLiftState() == Lift.LiftState.NULL
                && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.NULL
                && mRobot.mLift.getCurrentWristState() == Lift.WristState.NULL
                && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.NULL;

        boolean alreadyAttemptedGrab = mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HUMAN_PLAYER_POSITION
                && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.HUMAN_PLAYER_POSITION
                && mRobot.mLift.getCurrentWristState() == Lift.WristState.HUMAN_PLAYER_POSITION_2
                && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION;

        boolean inGrabbedState = mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HUMAN_PLAYER_POSITION_2
                && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.HUMAN_PLAYER_POSITION_2
                && mRobot.mLift.getCurrentWristState() == Lift.WristState.HUMAN_PLAYER_POSITION_2
                && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION;

        boolean inHPNormalButGrabbed = mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HUMAN_PLAYER_POSITION
                && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.HUMAN_PLAYER_POSITION
                && mRobot.mLift.getCurrentWristState() == Lift.WristState.HUMAN_PLAYER_POSITION
                && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION;

        // Does the driver request and are we in a state that can get us there?
        // if so, request human player alignment, intake retraction
        if (gamepad1.left_bumper
                && (inIntaking|| inSomething ||inStartOfTeleop || alreadyAttemptedGrab || inGrabbedState)){

            mRobot.mLift.takeover = false;
            mRobot.mLift.request(Lift.TopLevelState.HUMAN_PLAYER_POSITION);
            mRobot.mIntake.requestState(Intake.TopLevelState.RETRACTION);
            sampleInRobot = false;

            liftTimer.reset();
        } else if (inHPNormalButGrabbed && liftTimer.milliseconds() > 200 && sampleInRobot){
            mRobot.mLift.request(Lift.TopLevelState.HUMAN_PLAYER_2);
        }

        // if in human player position and driver request
        // grab specimen from wall
        if (gamepad1.right_bumper
                && mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HUMAN_PLAYER_POSITION){

            mRobot.mLift.request(Lift.TopLevelState.HUMAN_PLAYER_GRABED);

            mRobot.mLift.takeover = false;
            sampleInRobot = true;

            mRobot.mIntake.setIntakePower(Intake.PowerState.OFF.setTargetPower());
            liftTimer.reset();
        }

        boolean pickedUpSpec = mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HUMAN_PLAYER_POSITION_2
                && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.HUMAN_PLAYER_POSITION_2
                && mRobot.mLift.getCurrentWristState() == Lift.WristState.HUMAN_PLAYER_POSITION_2
                && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION;

        if (gamepad2.a && pickedUpSpec) {
            mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG);
        }

        boolean triedScoringSpec = mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HIGH_RUNG_POSITION
                && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.RUNG_SCORE_POSITION
                && mRobot.mLift.getCurrentWristState() == Lift.WristState.RUNG_SCORE_POSITION
                && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION;

        if (gamepad1.right_bumper && triedScoringSpec) {
            mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG_SCORED);
            something = false;
        }

        if (something && (mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HIGH_RUNG_SCORE_POSITION && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.RUNG_SCORE_POSITION && mRobot.mLift.getCurrentWristState() == Lift.WristState.RUNG_SCORE_POSITION && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION)){
            if(gamepad1.right_bumper) {
                mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG_RELEASE);
                sampleInRobot = false;
            }
            else if (gamepad1.left_bumper) {
                mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG);
                something = false;
            }
        } else {
            something = !gamepad1.right_bumper;
        }

        if ((mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HIGH_RUNG_RELEASED_POSITION && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.RUNG_SCORE_POSITION && mRobot.mLift.getCurrentWristState() == Lift.WristState.RUNG_SCORE_POSITION && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.OPEN_POSITION)) {
            mRobot.mLift.request(Lift.TopLevelState.TRANSFER_PREPARE);
        }

        if (gamepad1.a) {
            if (mRobot.mIntake.sampleInIntake()){
                mRobot.mLift.request(Lift.TopLevelState.TRANSFER_PREPARE);
                mRobot.mIntake.requestState(Intake.TopLevelState.RETRACTION);
                mRobot.mIntake.auto = false;
                sampleInRobot = true;
                score = false;
            } else {
                mRobot.mIntake.setFlickerPosition(sampleUnlock);
                mRobot.mIntake.requestState(Intake.TopLevelState.DEPLOY);
                mRobot.mLift.request(Lift.TopLevelState.TRANSFER_PREPARE);
                mRobot.mIntake.CurrentIntakeState = Intake.IntakeState.RETRACTED;
                mRobot.mIntake.CurrentWristState = Intake.WristState.UP_POSITION;
                mRobot.mIntake.CurrentPowerState = Intake.PowerState.OFF;
                mRobot.mIntake.auto = true;
            }
        } else if (gamepad1.b) {
            mRobot.mIntake.requestState(Intake.TopLevelState.RETRACTION);
            score = false;
        } else if(gamepad1.x) { // Outtake
            mRobot.mIntake.setFlickerPosition(sampleUnlock);
            mRobot.mIntake.requestState(Intake.TopLevelState.DEPLOYED_OUTTAKE);
            mRobot.mLift.request(Lift.TopLevelState.TRANSFER_PREPARE);

            mRobot.mIntake.CurrentIntakeState = Intake.IntakeState.RETRACTED;
            mRobot.mIntake.CurrentWristState = Intake.WristState.UP_POSITION;
            mRobot.mIntake.CurrentPowerState = Intake.PowerState.OFF;
            mRobot.mIntake.auto = true;
        }

        if(gamepad2.y) {
            mRobot.mIntake.setIntakePower(1);
            mRobot.mIntake.powerAllowed = false;
        } else if(gamepad2.x) {
            mRobot.mIntake.setIntakePower(-1);
            mRobot.mIntake.powerAllowed = false;
        } else
            mRobot.mIntake.powerAllowed = true;

        mRobot.mLift.update();


        telemetry.addData("Alliance Color", color);

        telemetry.addData("CurrentDraw", mRobot.mLift.getCurrentDraw());
        telemetry.addData("LastGoodState", mRobot.mLift.lastLiftStateThatWasGood());

        telemetry.addData("Lift Position", mRobot.mLift.currentLiftPosition());
        telemetry.addData("Intake Position", mRobot.mIntake.extension.getCurrentPosition());

        telemetry.addData("Current Intake State", mRobot.mIntake.getCurrentIntakeState());
        telemetry.addData("Requested Intake State", mRobot.mIntake.RequestedIntakeState);

        telemetry.addData("Current Intake Wrist State", mRobot.mIntake.getCurrentWristState());
        telemetry.addData("Requested Intake Wrist State", mRobot.mIntake.RequstedWristState);

        telemetry.addData("Current Intake Power State", mRobot.mIntake.getCurrentPowerState());
        telemetry.addData("Requested Intake Power State", mRobot.mIntake.RequstedPowerState);

        telemetry.addData("Current Intake Sample State", mRobot.mIntake.getIntakeColor());

        telemetry.addData("Current Lift State", mRobot.mLift.getCurrentLiftState());
        telemetry.addData("Requested Lift State", mRobot.mLift.RequestedLiftState);

        telemetry.addData("Current Lift Elbow State", mRobot.mLift.getCurrentElbowState());
        telemetry.addData("Requested Lift Elbow State", mRobot.mLift.RequestedElbowState);

        telemetry.addData("Current Lift Wrist State", mRobot.mLift.getCurrentWristState());
        telemetry.addData("Requested Lift Wrist State", mRobot.mLift.RequestedWristState);

        telemetry.addData("Current Lift Claw State", mRobot.mLift.getCurrentClawState());
        telemetry.addData("Requested Lift Claw State", mRobot.mLift.RequestedClawState);

        telemetry.addData("pose", mRobot.mDrive.getPose());


        telemetry.update();

        mRobot.mLift.setWristPosition(mRobot.mLift.RequestedWristState.getTargetPosition());

        mRobot.mIntake.update();
        mRobot.mDrive.update();
    }
}