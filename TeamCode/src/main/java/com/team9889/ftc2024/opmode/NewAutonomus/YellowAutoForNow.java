package com.team9889.ftc2024.opmode.NewAutonomus;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.team9889.ftc2024.subsystems.Intake;
import com.team9889.ftc2024.subsystems.Lift;
import com.team9889.ftc2024.subsystems.Robot;
import com.team9889.ftc2024.subsystems.SparkFunOTOSDrive;

@Autonomous(preselectTeleOp = "TeleOperate")
public class YellowAutoForNow extends LinearOpMode {
    Robot mRobot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(41 , 63, Math.toRadians(-180));

        mRobot.init(hardwareMap, beginPose);

        mRobot.mIntake.setIntakeWristPosition(Intake.WristState.UP_POSITION.getTargetPosition());

        waitForStart();

        Actions.runBlocking(mRobot.mIntake.Retracted());
        mRobot.mIntake.setRequstedPowerState(Intake.PowerState.ON);

        Actions.runBlocking(new SequentialAction(
                mRobot.mDrive.DriveYLess(56, new Vector2d(0, 0.4)),
                mRobot.mDrive.DriveX(50, new Vector2d(-0.4, 0)),
                mRobot.mDrive.DriveX(54, new Vector2d(-0.3, 0)),
                mRobot.mDrive.Rotate1(-100, 1),
                mRobot.mDrive.Rotate1(-100-4, -0.5),
                mRobot.mDrive.Rotate1(-100, 0.3),
                mRobot.mLift.scoreHigh())
        );

        Actions.runBlocking(mRobot.mDrive.DriveYLess(53, new Vector2d(0.3,0)));

        Actions.runBlocking(new ParallelAction(
                mRobot.mLift.liftRetract(),
                new SequentialAction(
                        new InstantAction(() -> {
                           mRobot.mIntake.setIntakeLockPosition(1);
                        }),
                        new SleepAction(0.25),
                        mRobot.mIntake.AutoSamples(577),
                        new SleepAction(0.5),
                        mRobot.mIntake.AutoSamples(400),
                        new SleepAction(0.5)
                ))
        );


        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                mRobot.mIntake.Retracted(),
                        new InstantAction(() -> {
                            mRobot.mIntake.setRequstedPowerState(Intake.PowerState.ON);
                        }),
                new InstantAction(() -> {
                    mRobot.mLift.setElbowPosition(Lift.ElbowStates.INTAKE_POSITION.getTargetPosition());
                    mRobot.mLift.setWristPosition(Lift.WristState.INTAKE_POSITION.getTargetPosition());
                    mRobot.mLift.setClawPosition(Lift.ClawStates.OPEN_POSITION.getTargetPosition());
                })),
                mRobot.mIntake.waitForRetract(),
                new SleepAction(0.5),
                new InstantAction(() -> {
                    mRobot.mLift.setElbowPosition(Lift.ElbowStates.TRANSFER_POSITION.getTargetPosition());
                    mRobot.mLift.setWristPosition(Lift.WristState.TRANSFER_POSITION.getTargetPosition());
                    mRobot.mLift.setClawPosition(Lift.ClawStates.OPEN_POSITION.getTargetPosition());
                }),
                new InstantAction(() -> {
                    mRobot.mLift.setElbowPosition(Lift.ElbowStates.TRANSFER_POSITION.getTargetPosition());
                    mRobot.mLift.setWristPosition(Lift.WristState.TRANSFER_POSITION.getTargetPosition());
                    mRobot.mLift.setClawPosition(Lift.ClawStates.CLOSED_POSITION.getTargetPosition());
                }),
                new SleepAction(0.5),
                new InstantAction(() ->  {
                mRobot.mIntake.setIntakePower(Intake.PowerState.OUTTAKE.setTargetPower());
                }),
                mRobot.mDrive.DriveY(55, new Vector2d(-0.3, 0)),
                new SleepAction(0.5),
                mRobot.mLift.scoreHigh(),
                new SleepAction(0.5),
                mRobot.mDrive.DriveYLess(54, new Vector2d(0.3, 0))
        ));

        double angle = -93.5;
        Actions.runBlocking(
                new ParallelAction(new SequentialAction(
                mRobot.mDrive.Rotate1(angle, 1),
                mRobot.mDrive.Rotate1(angle-6, -0.4),
                mRobot.mDrive.Rotate1(angle, 0.2)),
                mRobot.mLift.liftRetract(),
                new SleepAction(0.5),
                mRobot.mDrive.DriveX(60, new Vector2d(0, 0.3))
                )

        );

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        new InstantAction(() -> {
                            mRobot.mIntake.setIntakePower(Intake.PowerState.ON.setTargetPower());
                            mRobot.mIntake.setIntakeLockPosition(1);
                        }),
                        new SleepAction(0.25),
                        mRobot.mIntake.AutoSamples(530),
                        new SleepAction(0.5),
                        mRobot.mIntake.AutoSamples(400),
                        new SleepAction(0.5)
                ))
        );


        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        mRobot.mIntake.Retracted(),
                        new InstantAction(() -> {
                            mRobot.mIntake.setRequstedPowerState(Intake.PowerState.ON);
                        }),
                        new InstantAction(() -> {
                            mRobot.mLift.setElbowPosition(Lift.ElbowStates.INTAKE_POSITION.getTargetPosition());
                            mRobot.mLift.setWristPosition(Lift.WristState.INTAKE_POSITION.getTargetPosition());
                            mRobot.mLift.setClawPosition(Lift.ClawStates.OPEN_POSITION.getTargetPosition());
                        })),
                mRobot.mIntake.waitForRetract(),
                new SleepAction(0.5),
                new InstantAction(() -> {
                    mRobot.mLift.setElbowPosition(Lift.ElbowStates.TRANSFER_POSITION.getTargetPosition());
                    mRobot.mLift.setWristPosition(Lift.WristState.TRANSFER_POSITION.getTargetPosition());
                    mRobot.mLift.setClawPosition(Lift.ClawStates.OPEN_POSITION.getTargetPosition());
                }),
                new InstantAction(() -> {
                    mRobot.mLift.setElbowPosition(Lift.ElbowStates.TRANSFER_POSITION.getTargetPosition());
                    mRobot.mLift.setWristPosition(Lift.WristState.TRANSFER_POSITION.getTargetPosition());
                    mRobot.mLift.setClawPosition(Lift.ClawStates.CLOSED_POSITION.getTargetPosition());
                }),
                new SleepAction(0.5),
                new InstantAction(() ->  {
                    mRobot.mIntake.setIntakePower(Intake.PowerState.OUTTAKE.setTargetPower());
                }),

                mRobot.mDrive.DriveY(54, new Vector2d(-0.3, 0)),
                new SleepAction(0.5),
                new InstantAction(() -> {
                    mRobot.mLift.setWristPosition(0.6);
                }),
                mRobot.mDrive.DriveX(66   , new Vector2d(0, 0.3)),
                mRobot.mLift.scoreHigh()

        ));
        angle = -81.5;
        Actions.runBlocking(new SequentialAction(
                        mRobot.mDrive.Rotate1(angle, 0.4),
                        mRobot.mLift.liftRetract()
                )
        );
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        new InstantAction(() -> {
                            mRobot.mIntake.setIntakePower(Intake.PowerState.ON.setTargetPower());
                            mRobot.mIntake.setIntakeLockPosition(1);
                        }),

                        mRobot.mDrive.DriveYLess(53, new Vector2d(0.3, 0)),

                        new SleepAction(0.25),
                        mRobot.mIntake.AutoSamples(577),
                        new SleepAction(0.5),
                        mRobot.mIntake.AutoSamples(430),
                        new SleepAction(0.5),

                        mRobot.mDrive.DriveY(54, new Vector2d(-0.3, 0))
                ))
        );


        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        mRobot.mIntake.Retracted(),
                        new InstantAction(() -> {
                            mRobot.mIntake.setRequstedPowerState(Intake.PowerState.ON);
                        }),
                        new InstantAction(() -> {
                            mRobot.mLift.setElbowPosition(Lift.ElbowStates.INTAKE_POSITION.getTargetPosition());
                            mRobot.mLift.setWristPosition(Lift.WristState.INTAKE_POSITION.getTargetPosition());
                            mRobot.mLift.setClawPosition(Lift.ClawStates.OPEN_POSITION.getTargetPosition());
                        })),
                mRobot.mIntake.waitForRetract(),
                new SleepAction(0.5),
                new InstantAction(() -> {
                    mRobot.mLift.setElbowPosition(Lift.ElbowStates.TRANSFER_POSITION.getTargetPosition());
                    mRobot.mLift.setWristPosition(Lift.WristState.TRANSFER_POSITION.getTargetPosition());
                    mRobot.mLift.setClawPosition(Lift.ClawStates.OPEN_POSITION.getTargetPosition());
                }),
                new InstantAction(() -> {
                    mRobot.mLift.setElbowPosition(Lift.ElbowStates.TRANSFER_POSITION.getTargetPosition());
                    mRobot.mLift.setWristPosition(Lift.WristState.TRANSFER_POSITION.getTargetPosition());
                    mRobot.mLift.setClawPosition(Lift.ClawStates.CLOSED_POSITION.getTargetPosition());
                }),

                new SleepAction(0.5),
                new InstantAction(() ->  {
                    mRobot.mIntake.setIntakePower(Intake.PowerState.OUTTAKE.setTargetPower());
                })



        ));
        angle = -93.5;
        Actions.runBlocking(new SequentialAction(
                        mRobot.mDrive.Rotate1(angle, -1),
                        mRobot.mLift.scoreHigh(),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(1),
                                        mRobot.mLift.liftRetract()
                                ),
                                new SequentialAction(
                                        new InstantAction(() -> {
                                            mRobot.mFlag.setFlagPosition(0.53);
                                            mRobot.mIntake.setIntakeLockPosition(1);
                                            mRobot.mDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(1, 0), -0.15));
                                        }),
                                        new SleepAction(1.35),
                                        new InstantAction(() -> {mRobot.mDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));}))
                                )
                )
        );


        // :)

//        Actions.runBlocking(mRobot.mDrive.Rotate1(-100, 1));
//        Actions.runBlocking(mRobot.mDrive.Rotate1(-100-4, -0.5));
//        Actions.runBlocking(mRobot.mDrive.Rotate1(-100, -0.3));





//        Actions.runBlocking(
//                new ParallelAction(
//                        mRobot.mLift.LiftController(),
//                        new SequentialAction(
//                                mRobot.mLift.HighBasketReady(),
//                                mRobot.mLift.waitForLiftUp(),
//                                mRobot.mDrive.Rotate1(-175, 0.3),
//                                mRobot.mLift.HighBasketReadyScored()
////                                mRobot.mDrive.DriveY(60, new Vector2d(0.3, 0)),
////                                mRobot.mLift.TransferPrepare(),
////                                mRobot.mIntake.AutoSamples()
//                        )
//                )
//        );

//        Actions.runBlocking(mRobot.mDrive.DriveToPoint(new Pose2d(57, 54.7, Math.toRadians(-114))));

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
