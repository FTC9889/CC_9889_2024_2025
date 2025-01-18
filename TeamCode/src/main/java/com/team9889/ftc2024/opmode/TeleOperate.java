package com.team9889.ftc2024.opmode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team9889.ftc2024.subsystems.Intake;
import com.team9889.ftc2024.subsystems.Lift;
import com.team9889.ftc2024.subsystems.Robot;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TeleOperate extends LinearOpMode{
    Robot mRobot = new Robot();
    private List<Action> requestedActions = new ArrayList<>();
    boolean score = false;
    boolean sampleInRobot = false;
    boolean something = false;
    boolean clawReleased = false;
    boolean yellow = true;
    boolean press = false;
    String color = "Nothing";

    Intake.SampleColor allianceColor = Intake.SampleColor.RED;
    Intake.SampleColor opponentColor = Intake.SampleColor.BLUE;

    @Override
    public void runOpMode() throws InterruptedException {
        mRobot.init(hardwareMap);

        telemetry.addLine("Robot in Init");
        telemetry.update();

        waitForStart();
        Action intakeAction = null;
        Action liftAction = null;

        requestedActions.add(mRobot.mLift.LiftController());

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            if (gamepad2.back && gamepad2.x){
                allianceColor = Intake.SampleColor.BLUE;
                opponentColor = Intake.SampleColor.RED;
            }
            if (gamepad2.back && gamepad2.b){
                allianceColor = Intake.SampleColor.RED;
                opponentColor = Intake.SampleColor.BLUE;
            }

            if (!press && gamepad2.y && gamepad2.back){
                yellow = !yellow;
                press = true;
            } else
                press = false;

            if (!yellow) {
                mRobot.mFlag.setFlagPosition(0.85);
            }else {
                mRobot.mFlag.setFlagPosition(0.9);
            }

            // Send calculated power to wheels
            mRobot.mDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));

            if (mRobot.mIntake.allowDriverExtension()){
                if(Math.abs(gamepad1.right_trigger) > Math.abs(gamepad1.left_trigger))
                    mRobot.mIntake.setExtensionPower(gamepad1.right_trigger);
                else if(Math.abs(gamepad1.right_trigger) < Math.abs(gamepad1.left_trigger))
                    mRobot.mIntake.setExtensionPower(-gamepad1.left_trigger);
                else
                    mRobot.mIntake.setExtensionPower(0);
            }

            if (gamepad1.a) {
                intakeAction = mRobot.mIntake.Deployed();
                liftAction = mRobot.mLift.TransferPrepare();
            } else if (gamepad1.b) {
                intakeAction = mRobot.mIntake.Retracted();
                score = false;
            }

            if (mRobot.mIntake.getCurrentIntakeState() == Intake.IntakeState.INTAKE) {
                if (mRobot.mIntake.getIntakeColor() == allianceColor ||
                        (mRobot.mIntake.getIntakeColor() == Intake.SampleColor.NEUTRAL && yellow)) {

                    intakeAction = mRobot.mIntake.Retracted();
                    sampleInRobot = true;
                    score = false;
                }

                if ((((mRobot.mIntake.getIntakeColor() == Intake.SampleColor.NEUTRAL && !yellow) || mRobot.mIntake.getIntakeColor()  == opponentColor
                        && mRobot.mIntake.getCurrentWristState() == Intake.WristState.DOWN_POSITION) || gamepad1.y) ){
                    intakeAction = mRobot.mIntake.Outtake();
                }
            }

            if (!score) {
                if (mRobot.mIntake.getCurrentWristState() == Intake.WristState.UP_POSITION
                        && mRobot.mIntake.getCurrentIntakeState() == Intake.IntakeState.RETRACTED
                        && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.INTAKE_POSITION) {
                    liftAction = mRobot.mLift.TransferReady();
                } else if (mRobot.mIntake.getCurrentIntakeState() == Intake.IntakeState.RETRACTED
                        && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.TRANSFER_POSITION
                        && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.OPEN_POSITION) {
                    liftAction = mRobot.mLift.TransferComplete();
                    mRobot.mIntake.setIntakePower(Intake.PowerState.OFF.setTargetPower());
                } else if (mRobot.mIntake.getCurrentIntakeState() == Intake.IntakeState.RETRACTED
                        && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION
                        && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.TRANSFER_POSITION
                        && mRobot.mLift.getCurrentLiftState() == Lift.LiftState.TRANSFER_POSITION) {
                    liftAction = mRobot.mLift.ScorePrepare();
                }

                if (gamepad2.a && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION
                        && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.DEFAULT_POSITION) {
                    liftAction = mRobot.mLift.HighBasketReady();
                    intakeAction = mRobot.mIntake.Outtake();
                }

                if (gamepad1.right_bumper && mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HIGH_BASKET_POSITION) {
                    liftAction = mRobot.mLift.HighBasketReadyScored();
                    intakeAction = mRobot.mIntake.Retracted();
                    sampleInRobot = false;
                }

                if (mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HIGH_BASKET_POSITION && mRobot.mIntake.getCurrentIntakeState() == Intake.IntakeState.RETRACTED)
                    mRobot.mIntake.setIntakePower(0);
            }

            if(mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HIGH_BASKET_POSITION
                    && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.BASKET_SCORE_READY_POSITION
                    && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.OPEN_POSITION && !gamepad1.right_bumper && mRobot.mLift.getCurrentDraw() < 10000) {
                liftAction = mRobot.mLift.TransferPrepare();
                intakeAction = mRobot.mIntake.Retracted();
                score = true;
            }

            if (gamepad1.left_bumper
                    && (
                            (mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.INTAKE_POSITION && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.OPEN_POSITION)
                        ||mRobot.mLift.inStates(Lift.LiftState.DEFAULT_POSITION, Lift.ElbowStates.DEFAULT_POSITION, Lift.WristState.DEFAULT_POSITION, Lift.ClawStates.CLOSED_POSITION)
                        || mRobot.mLift.inStates(Lift.LiftState.NULL, Lift.ElbowStates.NULL, Lift.WristState.NULL, Lift.ClawStates.NULL)
                        || mRobot.mLift.inStates(Lift.LiftState.HUMAN_PLAYER_POSITION, Lift.ElbowStates.HUMAN_PLAYER_POSITION_2, Lift.WristState.HUMAN_PLAYER_POSITION_2, Lift.ClawStates.CLOSED_POSITION)
                    )
            ){
                liftAction = mRobot.mLift.HumanPlayer();
                intakeAction = mRobot.mIntake.Retracted();
            }

            if (gamepad1.right_bumper
                    && mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HUMAN_PLAYER_POSITION){
                liftAction = mRobot.mLift.HumanPlayerIntaked();
                sampleInRobot = true;
                mRobot.mIntake.setIntakePower(Intake.PowerState.OFF.setTargetPower());
            }

            if (mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HUMAN_PLAYER_POSITION
                    && mRobot.mLift.getCurrentWristState() == Lift.WristState.HUMAN_PLAYER_POSITION
                    && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION){
                liftAction = mRobot.mLift.HumanPlayerIntakedReady();
            }



            if (gamepad2.a && mRobot.mLift.inStates(Lift.LiftState.HUMAN_PLAYER_POSITION, Lift.ElbowStates.HUMAN_PLAYER_POSITION_2, Lift.WristState.HUMAN_PLAYER_POSITION_2, Lift.ClawStates.CLOSED_POSITION)) {
                liftAction = mRobot.mLift.HighRung();
            }

            if (gamepad1.right_bumper && mRobot.mLift.inStates(Lift.LiftState.HIGH_RUNG_POSITION, Lift.ElbowStates.RUNG_SCORE_POSITION, Lift.WristState.RUNG_SCORE_POSITION, Lift.ClawStates.CLOSED_POSITION)) {
                liftAction = mRobot.mLift.HighRungScored();
                something = false;
            }

            if (something && mRobot.mLift.inStates(Lift.LiftState.HIGH_RUNG_SCORE_POSITION, Lift.ElbowStates.RUNG_SCORE_POSITION, Lift.WristState.RUNG_SCORE_POSITION, Lift.ClawStates.CLOSED_POSITION)){
                if(gamepad1.right_bumper) {
                    liftAction = mRobot.mLift.HighRungScoredReleased();
                    sampleInRobot = false;
                }
                else if (gamepad1.left_bumper) {
                    liftAction = mRobot.mLift.HighRungScored();
                    something = false;
                }
            } else {
                something = !gamepad1.right_bumper;
            }

            if (mRobot.mLift.inStates(Lift.LiftState.HIGH_RUNG_RELEASED_POSITION, Lift.ElbowStates.RUNG_SCORE_POSITION, Lift.WristState.RUNG_SCORE_POSITION, Lift.ClawStates.OPEN_POSITION)) {
                liftAction = mRobot.mLift.TransferPrepare();
            }


            List<Action> newActions = new ArrayList<>();
            for (Action action : requestedActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
                telemetry.addData("Action", action.toString());
            }
            requestedActions = newActions;


            if (intakeAction != null) {
                intakeAction.preview(packet.fieldOverlay());
                if(!intakeAction.run(packet)) {
                    intakeAction = null;
                }
            }

            if (liftAction != null) {
                liftAction.preview(packet.fieldOverlay());
                if(!liftAction.run(packet)) {
                    liftAction = null;
                }
            }

            telemetry.addData("Alliance Color", color);

            telemetry.addData("CurrentDraw", mRobot.mLift.getCurrentDraw());
            telemetry.addData("LastGoodState", mRobot.mLift.lastLiftStateThatWasGood());

            telemetry.addData("Lift Position", mRobot.mLift.currentLiftPosition());

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


            telemetry.update();
        }
    }
}