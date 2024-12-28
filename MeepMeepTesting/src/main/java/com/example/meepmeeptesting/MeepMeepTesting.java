package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity SampleAuto = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-8, -60, Math.toRadians(-90)))
                        .strafeTo(new Vector2d(-8, -30))
                        .splineToLinearHeading(new Pose2d(-48, -40, Math.toRadians(90)), 90)
                        .splineToLinearHeading(new Pose2d(-58, -54, Math.toRadians(45)), 45)
                        .splineToLinearHeading(new Pose2d(-58, -40, Math.toRadians(90)), 90)
                        .splineToLinearHeading(new Pose2d(-58, -54, Math.toRadians(45)), 45)
                        .splineToLinearHeading(new Pose2d(-58, -40, Math.toRadians(120)), 90)
                        .splineToLinearHeading(new Pose2d(-58, -54, Math.toRadians(45)), 45)
                        .turn(Math.toRadians(-52))
                        .strafeTo(new Vector2d(48, -62))
                        .build());

        RoadRunnerBotEntity SpecimenAuto = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(-90)))
                        .strafeTo(new Vector2d(8, -30))
                        .splineToLinearHeading(new Pose2d(47, -48, Math.toRadians(90)), Math.toRadians(90))
                        .strafeTo(new Vector2d(58, -48))
                        .turn(Math.toRadians(-25))
                        .lineToLinearHeading(new Pose2d(35, -60, Math.toRadians(90)))
                        .splineToLinearHeading(new Pose2d(4, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(35, -60, Math.toRadians(90)), Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(0, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(35, -60, Math.toRadians(90)), Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(-4, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(35, -60, Math.toRadians(90)), Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(-8, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(40, -60, Math.toRadians(0)), Math.toRadians(-90))

                        .build());


        RoadRunnerBotEntity JustPark = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(24, -60, Math.toRadians(0)))
                        .lineTo(new Vector2d(40, -60))
                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(SampleAuto)
                .addEntity(SpecimenAuto)
                .addEntity(JustPark)
                .start();
    }
}