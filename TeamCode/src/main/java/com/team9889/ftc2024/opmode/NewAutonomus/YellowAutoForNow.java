package com.team9889.ftc2024.opmode.NewAutonomus;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.team9889.ftc2024.subsystems.Robot;
import com.team9889.ftc2024.subsystems.SparkFunOTOSDrive;

@Autonomous
public class YellowAutoForNow extends LinearOpMode {
    Robot mRobot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(36 , 63, Math.toRadians(-180));

        mRobot.init(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(mRobot.mDrive.DriveToPoint(new Pose2d(57, 54.7, Math.toRadians(-114))));

//        while (opModeIsActive() && mRobot.mDrive.pose.position.x < 57) {
//            mRobot.mDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.5, 0), 0));
//            mRobot.mDrive.updatePoseEstimate();
//        }
//        mRobot.mDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

//
//        while (opModeIsActive()) {
//            Actions.runBlocking(
//                    mRobot.mDrive.actionBuilder(beginPose)
//                            .strafeToLinearHeading(new Vector2d(57, 54.7), Math.toRadians(-114))
//                            .strafeToLinearHeading(new Vector2d(36, 63), Math.toRadians(-180))
//                            .build());
//        }

//        Actions.runBlocking(mRobot.mDrive.actionBuilder(beginPose)
//                .strafeToLinearHeading(new Vector2d(57, 54.7), Math.toRadians(-114)).build()
//        );

//        Actions.runBlocking(
//                new ParallelAction(
//                        mRobot.mLift.LiftController(),
//                mRobot.mDrive.actionBuilder(beginPose)
//                .stopAndAdd(new ParallelAction(
//                        mRobot.mIntake.Retracted(),
//                        mRobot.mLift.HighBasketReady()
//                ))
//                .strafeToLinearHeading(new Vector2d(57, 54.7), Math.toRadians(-114))
//                .stopAndAdd(new SequentialAction(
//                        mRobot.mLift.HighBasketReadyScored(),
//                                new ParallelAction(
//                                        mRobot.mIntake.AutoSamples(),
//                                        mRobot.mLift.TransferPrepare()
//                                ),
//                                mRobot.mIntake.Retracted(),
//                                new SequentialAction(
//                                        mRobot.mLift.TransferReady(),
//                                        mRobot.mLift.TransferComplete(),
//                                        mRobot.mLift.HighBasketReady(),
//                                        mRobot.mLift.HighBasketReadyScored()
//                                )))
//
//                .strafeToLinearHeading(new Vector2d(59.3, 52), Math.toRadians(-99.3))
//                .afterDisp(1, mRobot.mLift.TransferPrepare())
//                .stopAndAdd(new SequentialAction(
//                        new ParallelAction(
//                            mRobot.mIntake.AutoSamples(),
//                            mRobot.mLift.TransferPrepare()),
//                        mRobot.mIntake.Retracted()
//                    )
//                )
//                .build())
//        );
//
//
//        Actions.runBlocking(new ParallelAction(
//                        mRobot.mLift.LiftController(),
//
//
//                                ,
//                                mRobot.mIntake.Retracted(),
//                                new ParallelAction(
//                                        mRobot.mDrive.actionBuilder(beginPose).
//                                                strafeToLinearHeading(new Vector2d(57, 54.7), Math.toRadians(-114)).build(),
//                                        new SequentialAction(
//                                                mRobot.mLift.TransferReady(),
//                                                mRobot.mLift.TransferComplete(),
//                                                mRobot.mLift.HighBasketReady(),
//                                                mRobot.mLift.HighBasketReadyScored()
//                                        )
//                                ),
//                                new ParallelAction(
//                                        mRobot.mLift.TransferPrepare(),
//                                        mRobot.mDrive.actionBuilder(mRobot.mDrive.pose).
//                                                strafeToLinearHeading(new Vector2d(57.5, 50.6), Math.toRadians(-70.3)).build()
//
//                                ),
//
//                                new ParallelAction(
//                                        mRobot.mIntake.AutoSamples(),
//                                        mRobot.mLift.TransferPrepare()
//                                ),
//                                mRobot.mIntake.Retracted(),
//                                new ParallelAction(
//                                        mRobot.mDrive.actionBuilder(beginPose).
//                                                strafeToLinearHeading(new Vector2d(57, 54.7), Math.toRadians(-114)).build(),
//                                        new SequentialAction(
//                                                mRobot.mLift.TransferReady(),
//                                                mRobot.mLift.TransferComplete(),
//                                                mRobot.mLift.HighBasketReady(),
//                                                mRobot.mLift.HighBasketReadyScored()
//                                        )
//                                )
//
//
//                        )
//                )
//
//

//        Actions.runBlocking(new ParallelAction(
//                mRobot.mLift.LiftController(),
//                    new SequentialAction(
//                            new ParallelAction(
//                                    mRobot.mIntake.Retracted(),
//                                    mRobot.mLift.HighBasketReady()
//                            ),
//                            mRobot.mDrive.actionBuilder(beginPose).
//                                    strafeToLinearHeading(new Vector2d(57, 54.7), Math.toRadians(-114)).build(),
//                            mRobot.mLift.HighBasketReadyScored(),
//                            new ParallelAction(
//                                    mRobot.mIntake.AutoSamples(),
//                                    mRobot.mLift.TransferPrepare()
//                            ),
//                            mRobot.mIntake.Retracted(),
//                            new SequentialAction(
//                                    mRobot.mLift.TransferReady(),
//                                    mRobot.mLift.TransferComplete(),
//                                    mRobot.mLift.HighBasketReady(),
//                                    mRobot.mLift.HighBasketReadyScored()
//                            ),
//                            new ParallelAction(
//                                    mRobot.mLift.TransferPrepare(),
//                                    mRobot.mDrive.actionBuilder(mRobot.mDrive.pose).
//                                            strafeToLinearHeading(new Vector2d(59.3, 52), Math.toRadians(-99.3)).build()
//                            ),
//
//                            new ParallelAction(
//                                    mRobot.mIntake.AutoSamples(),
//                                    mRobot.mLift.TransferPrepare()
//                            ),
//                            mRobot.mIntake.Retracted(),
//                            new ParallelAction(
//                                    mRobot.mDrive.actionBuilder(beginPose).
//                                            strafeToLinearHeading(new Vector2d(57, 54.7), Math.toRadians(-114)).build(),
//                                    new SequentialAction(
//                                            mRobot.mLift.TransferReady(),
//                                            mRobot.mLift.TransferComplete(),
//                                            mRobot.mLift.HighBasketReady(),
//                                            mRobot.mLift.HighBasketReadyScored()
//                                    )
//                            ),
//                            new ParallelAction(
//                                    mRobot.mLift.TransferPrepare(),
//                                    mRobot.mDrive.actionBuilder(mRobot.mDrive.pose).
//                                            strafeToLinearHeading(new Vector2d(57.5, 50.6), Math.toRadians(-70.3)).build()
//
//                            ),
//
//                            new ParallelAction(
//                                    mRobot.mIntake.AutoSamples(),
//                                    mRobot.mLift.TransferPrepare()
//                            ),
//                            mRobot.mIntake.Retracted(),
//                            new ParallelAction(
//                                    mRobot.mDrive.actionBuilder(beginPose).
//                                            strafeToLinearHeading(new Vector2d(57, 54.7), Math.toRadians(-114)).build(),
//                                    new SequentialAction(
//                                            mRobot.mLift.TransferReady(),
//                                            mRobot.mLift.TransferComplete(),
//                                            mRobot.mLift.HighBasketReady(),
//                                            mRobot.mLift.HighBasketReadyScored()
//                                    )
//                            )
//
//
//                    )
//                )
//
//        );





    }
}
