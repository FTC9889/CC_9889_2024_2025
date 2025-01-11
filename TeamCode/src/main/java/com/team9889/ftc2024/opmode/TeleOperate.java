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

            // Send calculated power to wheels
            mRobot.mDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));

            if (mRobot.mIntake.allowDriverExtension()){
                if(Math.abs(gamepad1.right_trigger) > Math.abs(gamepad1.left_trigger))
                    mRobot.mIntake.setExtensionPower(gamepad1.right_trigger);
                else if(Math.abs(gamepad1.right_trigger) > Math.abs(gamepad1.left_trigger))
                    mRobot.mIntake.setExtensionPower(-gamepad1.left_trigger);
                else
                    mRobot.mIntake.setExtensionPower(0);
            }

            if (gamepad1.a) {
                intakeAction = mRobot.mIntake.Deployed();
                liftAction = mRobot.mLift.TransferPrepare();
            } else if (gamepad1.y) {
                intakeAction = mRobot.mIntake.Retracted() ;
            }

            if (mRobot.mIntake.getCurrentIntakeState() == Intake.IntakeState.INTAKE) {
                if (mRobot.mIntake.getIntakeColor() == Intake.SampleColor.BLUE
                        || mRobot.mIntake.getIntakeColor() == Intake.SampleColor.NEUTRAL){
                    intakeAction = mRobot.mIntake.Retracted();
                }
            }

            if(mRobot.mIntake.getCurrentWristState() == Intake.WristState.UP_POSITION
                    && mRobot.mIntake.getCurrentIntakeState() == Intake.IntakeState.RETRACTED
                    && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.INTAKE_POSITION) {
                liftAction = mRobot.mLift.TransferReady();
            } else if(mRobot.mIntake.getCurrentIntakeState() == Intake.IntakeState.RETRACTED
                    && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.TRANSFER_POSITION
                    && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.OPEN_POSITION) {
                liftAction = mRobot.mLift.TransferComplete();
                mRobot.mIntake.setIntakePower(Intake.PowerState.OFF.setTargetPower());
            } else if(mRobot.mIntake.getCurrentIntakeState() == Intake.IntakeState.RETRACTED
                    && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION
                    && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.TRANSFER_POSITION
                    && mRobot.mLift.getCurrentLiftState() == Lift.LiftState.TRANSFER_POSITION) {
                liftAction = mRobot.mLift.ScorePrepare();
            }

            if (gamepad2.a && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.CLOSED_POSITION
                    && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.DEFAULT_POSITION) {
                liftAction = mRobot.mLift.HighBasketReady();
            }

            if(gamepad1.right_bumper && mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HIGH_BASKET_POSITION) {
                liftAction = mRobot.mLift.HighBasketReadyScored();
            }

            if(mRobot.mLift.getCurrentLiftState() == Lift.LiftState.HIGH_BASKET_POSITION
                    && mRobot.mLift.getCurrentElbowState() == Lift.ElbowStates.BASKET_SCORE_READY_POSITION
                    && mRobot.mLift.getCurrentClawState() == Lift.ClawStates.OPEN_POSITION && !gamepad1.right_bumper) {
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