package com.team9889.ftc2024.opmode.NewAutonomus;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.team9889.ftc2024.subsystems.Robot;

@Autonomous
public class YellowAuto extends LinearOpMode {
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
                        .strafeTo(new Vector2d(-8, -30))
                        .afterDisp(5, mRobot.mLift.HighRung())
                        .stopAndAdd(mRobot.mLift.HighRungScored())
                        .stopAndAdd(mRobot.mLift.HighRungScoredReleased())
                        .splineToLinearHeading(new Pose2d(-48, -40, Math.toRadians(90)), 90)
                        .afterDisp(5, mRobot.mLift.TransferPrepare())
                        .stopAndAdd(mRobot.mIntake.AutoSamples())
                        .waitSeconds(0.25)
                        .splineToLinearHeading(new Pose2d(-58, -54, Math.toRadians(45)), 45)
                        .afterDisp(1, mRobot.mIntake.Retracted())
                        .afterDisp(14, mRobot.mLift.TransferReady())
                        .afterDisp(15, mRobot.mLift.TransferComplete())
                        .afterDisp(20, mRobot.mLift.HighBasket())
                        .stopAndAdd(mRobot.mLift.HighBasketReady())
                        .stopAndAdd(mRobot.mLift.HighBasketReadyScored())
                        .waitSeconds(0.5)
                        .splineToLinearHeading(new Pose2d(-58, -40, Math.toRadians(90)), 90)
                        .afterDisp(1, mRobot.mLift.TransferPrepare())
                        .afterDisp(1, mRobot.mIntake.AutoSamples())
                        .splineToLinearHeading(new Pose2d(-58, -54, Math.toRadians(45)), 45)
                        .afterDisp(1, mRobot.mIntake.Retracted())
                        .afterDisp(14, mRobot.mLift.TransferReady())
                        .afterDisp(15, mRobot.mLift.TransferComplete())
                        .afterDisp(20, mRobot.mLift.HighBasket())
                        .stopAndAdd(mRobot.mLift.HighBasketReady())
                        .stopAndAdd(mRobot.mLift.HighBasketReadyScored())
                        .waitSeconds(0.25)
                        .splineToLinearHeading(new Pose2d(-58, -40, Math.toRadians(120)), 90)
                        .afterDisp(1, mRobot.mLift.TransferPrepare())
                        .afterDisp(1, mRobot.mIntake.AutoSamples())
                        .splineToLinearHeading(new Pose2d(-58, -54, Math.toRadians(45)), 45)
                        .afterDisp(1, mRobot.mIntake.Retracted())
                        .afterDisp(14, mRobot.mLift.TransferReady())
                        .afterDisp(15, mRobot.mLift.TransferComplete())
                        .afterDisp(20, mRobot.mLift.HighBasket())
                        .stopAndAdd(mRobot.mLift.HighBasketReady())
                        .stopAndAdd(mRobot.mLift.HighBasketReadyScored())
                        .waitSeconds(0.25)
                        .stopAndAdd(mRobot.mLift.TransferPrepare())
                        .splineToLinearHeading(new Pose2d(-20, 0, Math.toRadians(180)), 0)
                        .stopAndAdd(mRobot.mLift.LevelOneAssent())
                        .build()
        );





    }
}
