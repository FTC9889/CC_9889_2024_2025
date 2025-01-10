package com.team9889.ftc2024.opmode.NewAutonomus;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.team9889.ftc2024.subsystems.Robot;

public class HumanPlayerAuto extends LinearOpMode {
    Robot mRobot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
//******//DON'T CHANGE THIS YOU WILL REGRET IT//**********************************************************************************************************//////////////////////////////////********************
        Pose2d beginPose = new Pose2d(0, 0, 0);
//********************************************************************************************

        mRobot.init(hardwareMap, beginPose);

        mRobot.mIntake.Retracted();

        waitForStart();

        Actions.runBlocking(
                mRobot.mDrive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(8, -30))
                        .afterDisp(5, mRobot.mLift.HighRung())
                        .stopAndAdd(mRobot.mLift.HighRungScored())
                        .stopAndAdd(mRobot.mLift.HighRungScoredReleased())
                        .splineToLinearHeading(new Pose2d(47, -48, Math.toRadians(90)), Math.toRadians(90))
                        .afterDisp(1, mRobot.mLift.TransferPrepare())
                        .afterDisp(20, mRobot.mIntake.AutoSpecimenSamples())
                        .waitSeconds(0.25)
                        .stopAndAdd(mRobot.mIntake.Retracted())
                        .stopAndAdd(mRobot.mLift.TransferReady())
                        .stopAndAdd(mRobot.mLift.TransferComplete())
                        .strafeTo(new Vector2d(58, -48))
                        .afterDisp(1, mRobot.mLift.ScorePrepare())
                        .afterDisp(10, mRobot.mLift.AutoDrop())
                        .afterDisp(10, mRobot.mIntake.AutoSpecimenSamples())
                        .waitSeconds(0.25)
                        .stopAndAdd(mRobot.mIntake.Retracted())
                        .waitSeconds(0.25)
                        .stopAndAdd(mRobot.mLift.TransferReady())
                        .stopAndAdd(mRobot.mLift.TransferComplete())
                        .stopAndAdd(mRobot.mLift.ScorePrepare())
                        .waitSeconds(0.25)
                        .stopAndAdd(mRobot.mLift.AutoDrop())
                        .turn(Math.toRadians(-25))
                        .afterDisp(10, mRobot.mIntake.AutoSpecimenSamples())
                        .waitSeconds(0.25)
                        .stopAndAdd(mRobot.mIntake.Retracted())
                        .stopAndAdd(mRobot.mLift.TransferReady())
                        .stopAndAdd(mRobot.mLift.TransferComplete())
                        .turn(Math.toRadians(25))
                        .stopAndAdd(mRobot.mLift.ScorePrepare())
                        .stopAndAdd(mRobot.mLift.AutoDrop())
                        .strafeTo(new Vector2d(35, -60))
                        .afterDisp(5, mRobot.mLift.HumanPlayer())
                        .stopAndAdd(mRobot.mLift.HumanPlayerIntaked())
                        .splineToLinearHeading(new Pose2d(4, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .afterDisp(0.001, mRobot.mLift.HighRung())
                        .stopAndAdd(mRobot.mLift.HighRungScored())
                        .stopAndAdd(mRobot.mLift.HighRungScoredReleased())
                        .splineToLinearHeading(new Pose2d(35, -60, Math.toRadians(90)), Math.toRadians(-90))
                        .afterDisp(5, mRobot.mLift.HumanPlayer())
                        .stopAndAdd(mRobot.mLift.HumanPlayerIntaked())
                        .waitSeconds(0.15)
                        .splineToLinearHeading(new Pose2d(0, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .afterDisp(0.001, mRobot.mLift.HighRung())
                        .stopAndAdd(mRobot.mLift.HighRungScored())
                        .stopAndAdd(mRobot.mLift.HighRungScoredReleased())
                        .splineToLinearHeading(new Pose2d(35, -60, Math.toRadians(90)), Math.toRadians(-90))
                        .afterDisp(5, mRobot.mLift.HumanPlayer())
                        .stopAndAdd(mRobot.mLift.HumanPlayerIntaked())
                        .waitSeconds(0.15)
                        .splineToLinearHeading(new Pose2d(-4, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .afterDisp(0.001, mRobot.mLift.HighRung())
                        .stopAndAdd(mRobot.mLift.HighRungScored())
                        .stopAndAdd(mRobot.mLift.HighRungScoredReleased())
                        .splineToLinearHeading(new Pose2d(35, -60, Math.toRadians(90)), Math.toRadians(-90))
                        .afterDisp(5, mRobot.mLift.HumanPlayer())
                        .stopAndAdd(mRobot.mLift.HumanPlayerIntaked())
                        .waitSeconds(0.15)
                        .splineToLinearHeading(new Pose2d(-8, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .afterDisp(0.001, mRobot.mLift.HighRung())
                        .stopAndAdd(mRobot.mLift.HighRungScored())
                        .stopAndAdd(mRobot.mLift.HighRungScoredReleased())
                        .splineToLinearHeading(new Pose2d(30, -46, Math.toRadians(-45)), Math.toRadians(-90))
                        .build());
    }
}
