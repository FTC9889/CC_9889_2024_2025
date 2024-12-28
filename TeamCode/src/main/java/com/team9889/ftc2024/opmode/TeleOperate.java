package com.team9889.ftc2024.opmode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

        waitForStart();

        requestedActions.add(mRobot.mLift.LiftController());

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Send calculated power to wheels
            mRobot.mDrive.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x / 0.66);

            if (gamepad2.a){
                requestedActions.add(mRobot.mLift.HighRung());
            }




            List<Action> newActions = new ArrayList<>();
            for (Action action : requestedActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            requestedActions = newActions;



//            if (gamepad1.a){
//                //are there encoders on intake extension motors
//                mRobot.mIntake.extension.setTargetPosition();
//                mRobot.mIntake.extension.setPower(0.5);
//                mRobot.mIntake.extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                mRobot.mIntake.setWristPosision();
//            }

            //gamepad1.a = intake slight extend && intake wrist down && power intake          while ( until detect color or if gamepad1.y  is clicked      if color is opponent eject      if color is yellow or alliance color reset robot and transfer) { gamepad1 can change wrist adn intake  slide position with gamepad1.triggrs and wrist with gamepad1.bumpers
            //gamepad1.x = claw
            //gamepad1.triggers = intake slides
            //gamepad1.y = reset robot


            // gamepad2.y = reset robot
            // gamepad2.triggers = lift slides









//            if (gamepad1.a){
//                if (mRobot.mIntake.CurrentIntakeState == Intake.IntakeState.INTAKE && mRobot.mIntake.CurrentWristState == Intake.WristState.DOWN_POSITION){
//                    mRobot.mIntake.CurrentWristState = Intake.WristState.UP_POSITION;
//                    mRobot.mIntake.CurrentIntakeState = Intake.IntakeState.RETRACTED;
//                }while (mRobot.mIntake.CurrentSampleColor == Intake.SampleColor.NULL) {
//                    mRobot.mIntake.RequestedIntakeState = Intake.IntakeState.INTAKE;
//                    mRobot.mIntake.RequstedWristState = Intake.WristState.DOWN_POSITION;
//                    if (gamepad1.right_trigger > gamepad1.left_trigger){
//                        mRobot.mIntake.extension.setTargetPosition();
//                        mRobot.mIntake.extension.setPower(0.5);
//                        mRobot.mIntake.extension.setTargetPosition();
//                    }if (gamepad1.right_trigger < gamepad1.left_trigger){
//
//                        mRobot.mIntake.extension.setPower(0.5);
//                        mRobot.mIntake.extension.setTargetPosition();
//                    }
//                }if (mRobot.mIntake.CurrentSampleColor == Intake.SampleColor.BLUE || mRobot.mIntake.CurrentSampleColor == Intake.SampleColor.RED || mRobot.mIntake.CurrentSampleColor == Intake.SampleColor.NEUTRAL){
//                    mRobot.mIntake.CurrentWristState = Intake.WristState.UP_POSITION;
//                    mRobot.mIntake.CurrentIntakeState = Intake.IntakeState.RETRACTED;
//                }
//            }

            if (mRobot.mIntake.RequestedIntakeState != mRobot.mIntake.CurrentIntakeState) {
                switch (mRobot.mIntake.RequestedIntakeState) {
                    case INTAKE:
                        switch (mRobot.mIntake.CurrentIntakeState) {
                            case AUTO_EXTEND:
                            case NULL:
                            case RETRACTED:
                                mRobot.mIntake.extension.setPower(0.5);
                                mRobot.mIntake.extension.setTargetPosition();
                                mRobot.mIntake.CurrentIntakeState = Intake.IntakeState.INTAKE;
                                break;
                            default:
                                break;
                        }
                        break;
//                    case OUTTAKE:
//                        switch (mRobot.mIntake.RequestedIntakeState) {
//                            case INTAKE:
//                                mRobot.mIntake.outtake();
//                                mRobot.mIntake.CurrentIntakeState = Intake.IntakeState.OUTTAKE;
//                                break;
//                            case RETRACTED:
//                                if (!mRobot.mIntake.digitalTouch.getState()) {
//                                    mRobot.mIntake.setPower(1);
//                                    allowDriverInputIntakeExtend = false;
//                                } else {
//                                    mRobot.mIntake.vfbDown();
//                                    mRobot.mIntake.out();
//                                    allowDriverInputIntakeExtend = true;
//                                    currentIntakeState = IntakeState.OUTTAKE;
//                                }
//                                break;
//                            default:
//                                break;
//                        }
//                        break;
                    case RETRACTED:
                        if (mRobot.mLift.CurrentLiftState != Lift.LiftState.DEFAULT_POSITION) {

                            mRobot.mLift.closedClaw();
                            mRobot.mLift.RequestedLiftState = Lift.LiftState.DEFAULT_POSITION;
                        }
                        switch (mRobot.mIntake.CurrentIntakeState) {
                            case AUTO_EXTEND:
                            case NULL:
                            case INTAKE:
                                mRobot.mIntake.RequstedWristState = Intake.WristState.UP_POSITION;
                                mRobot.mIntake.extension.setPower(0.5);
                                mRobot.mIntake.extension.setTargetPosition(0);
                                mRobot.mIntake.CurrentIntakeState = Intake.IntakeState.RETRACTED;
                                break;
                            default:
                                break;
                        }
                        break;
                    case AUTO_EXTEND:

                    case NULL:

                }
            }




        }
    }
}