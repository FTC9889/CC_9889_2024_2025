package com.team9889.ftc2024.test;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.team9889.ftc2024.subsystems.Robot;

@Autonomous
public class AutoTest extends LinearOpMode {
    Robot mRobot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(36 , 63, Math.toRadians(-180));

        mRobot.init(hardwareMap, beginPose);

        mRobot.mIntake.Retracted();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                mRobot.mDrive.actionBuilder(beginPose).
                        strafeToLinearHeading(new Vector2d(57, 54.7), Math.toRadians(-114))
                        .strafeToLinearHeading(new Vector2d(59.3, 52), Math.toRadians(-99.3))
                .build())
        );

    }
}
