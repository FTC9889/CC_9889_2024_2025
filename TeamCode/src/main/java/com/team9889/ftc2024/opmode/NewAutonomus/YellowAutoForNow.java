package com.team9889.ftc2024.opmode.NewAutonomus;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.team9889.ftc2024.subsystems.Robot;

@Autonomous
public class YellowAutoForNow extends LinearOpMode {
    Robot mRobot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
//******//DON'T CHANGE THIS YOU WILL REGRET IT//**********************************************************************************************************//////////////////////////////////********************
        Pose2d beginPose = new Pose2d(36 , 63, Math.toRadians(-180));
//********************************************************************************************

        mRobot.init(hardwareMap, beginPose);


        mRobot.mIntake.Retracted();


        waitForStart();


        Actions.runBlocking(new ParallelAction(
                mRobot.mLift.LiftController(),
                    new SequentialAction(
                            new ParallelAction(
                                    mRobot.mIntake.Retracted(),
                                    mRobot.mLift.HighBasketReady()
                            ),
                            mRobot.mDrive.actionBuilder(beginPose).
                                    strafeToLinearHeading(new Vector2d(57, 54.7), Math.toRadians(-114)).build(),
                            mRobot.mLift.HighBasketReadyScored(),
                            new ParallelAction(
                                    mRobot.mIntake.AutoSamples(),
                                    mRobot.mLift.TransferPrepare()
                            ),
                            mRobot.mIntake.Retracted(),
                            new SequentialAction(
                                    mRobot.mLift.TransferReady(),
                                    mRobot.mLift.TransferComplete(),
                                    mRobot.mLift.HighBasketReady(),
                                    mRobot.mLift.HighBasketReadyScored()
                            ),
                            new ParallelAction(
                                    mRobot.mLift.TransferPrepare(),
                                    mRobot.mDrive.actionBuilder(mRobot.mDrive.pose).
                                            strafeToLinearHeading(new Vector2d(59.3, 52), Math.toRadians(-99.3)).build()
                            ),

                            new ParallelAction(
                                    mRobot.mIntake.AutoSamples(),
                                    mRobot.mLift.TransferPrepare()
                            ),
                            mRobot.mIntake.Retracted(),
                            new ParallelAction(
                                    mRobot.mDrive.actionBuilder(beginPose).
                                            strafeToLinearHeading(new Vector2d(57, 54.7), Math.toRadians(-114)).build(),
                                    new SequentialAction(
                                            mRobot.mLift.TransferReady(),
                                            mRobot.mLift.TransferComplete(),
                                            mRobot.mLift.HighBasketReady(),
                                            mRobot.mLift.HighBasketReadyScored()
                                    )
                            ),
                            new ParallelAction(
                                    mRobot.mLift.TransferPrepare(),
                                    mRobot.mDrive.actionBuilder(mRobot.mDrive.pose).
                                            strafeToLinearHeading(new Vector2d(57.5, 50.6), Math.toRadians(-70.3)).build()

                            ),

                            new ParallelAction(
                                    mRobot.mIntake.AutoSamples(),
                                    mRobot.mLift.TransferPrepare()
                            ),
                            mRobot.mIntake.Retracted(),
                            new ParallelAction(
                                    mRobot.mDrive.actionBuilder(beginPose).
                                            strafeToLinearHeading(new Vector2d(57, 54.7), Math.toRadians(-114)).build(),
                                    new SequentialAction(
                                            mRobot.mLift.TransferReady(),
                                            mRobot.mLift.TransferComplete(),
                                            mRobot.mLift.HighBasketReady(),
                                            mRobot.mLift.HighBasketReadyScored()
                                    )
                            )


                    )
                )

        );





    }
}
