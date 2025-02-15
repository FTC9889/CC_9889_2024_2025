package com.team9889.ftc2024.opmode;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2024.subsystems.Intake;
import com.team9889.ftc2024.subsystems.Lift;
import com.team9889.ftc2024.subsystems.Robot;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name = "SpecimenAuto", group = "Examples")
public class SpecimenAuto extends OpMode {
    Robot mRobot = new Robot();

//    private Follower follower;

    private DashboardPoseTracker dashboardPoseTracker;

    private Timer pathTimer, actionTimer, opmodeTimer;


    private int pathState;

    ElapsedTime timer = new ElapsedTime();

    private final Pose startPose = new Pose(7.000, 63.740, Math.toRadians(-180));

    private final Pose scorePoint = new Pose(13.01, 129.65, Math.toRadians(-45));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreload;
    private PathChain firstSample, firstTurn, secondSample, secondTurn, thirdSample, thridTurn, firstIntakeReadyPosition, firstIntakeCompletePosition, firstScore, secondIntakeReadyPosition, secondIntakeCompletePosition, secondScore, thirdScore, fourthScore, fifthScore, park;


    public void buildPaths() {


        scorePreload = mRobot.mDrive.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(7.000, 63.740, Point.CARTESIAN),
                                new Point(39.212, 63.740, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();


        firstSample = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(39.212, 63.740, Point.CARTESIAN),
                                new Point(8.009, 63.740, Point.CARTESIAN),
                                new Point(32.538, 37.043, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        firstTurn = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(32.538, 37.043, Point.CARTESIAN),
                                new Point(32.538, 37.043, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(-135))
                .build();

        secondSample = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(32.538, 37.043, Point.CARTESIAN),
                                new Point(32.538, 37.043, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-65))
                .build();

        secondTurn = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(32.538, 37.043, Point.CARTESIAN),
                                new Point(32.538, 37.043, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-65), Math.toRadians(-135))
                .build();

        thirdSample = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(32.538, 37.043, Point.CARTESIAN),
                                new Point(32.538, 29.034, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-70))
                .build();

        thridTurn = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(32.538, 29.034, Point.CARTESIAN),
                                new Point(32.538, 29.034, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-70), Math.toRadians(-135))
                .build();

        firstIntakeReadyPosition = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(32.538, 29.034, Point.CARTESIAN),
                                new Point(10.846, 29.034, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(0))
                .build();

        firstIntakeCompletePosition = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(10.846, 29.034, Point.CARTESIAN),
                                new Point(8.700, 29.034, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        firstScore = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(8.700, 29.034, Point.CARTESIAN),
                                new Point(39.212, 66.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        secondIntakeReadyPosition = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(39.212, 66.000, Point.CARTESIAN),
                                new Point(10.846, 35.900, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        secondIntakeCompletePosition = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(10.846, 35.900, Point.CARTESIAN),
                                new Point(8.700, 35.900, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        secondScore = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(8.700, 29.034, Point.CARTESIAN),
                                new Point(39.212, 66.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        thirdScore = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(8.700, 35.900, Point.CARTESIAN),
                                new Point(39.212, 68.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        fourthScore = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 16
                        new BezierLine(
                                new Point(8.700, 35.900, Point.CARTESIAN),
                                new Point(39.212, 70.100, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        fifthScore = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 19
                        new BezierLine(
                                new Point(8.700, 35.900, Point.CARTESIAN),
                                new Point(39.212, 72.300, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        park = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 20
                        new BezierLine(
                                new Point(39.212, 72.300, Point.CARTESIAN),
                                new Point(21.692, 47.889, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-135))
                .build();
    }


    int sampleNumber = 0;

    public ArrayList<PathChain> intakeList;

    public ArrayList<PathChain> scoreList;

    public ArrayList<Intake.TopLevelState> sampleList ;

    ElapsedTime intakeTimer = new ElapsedTime();


    boolean up = true;

    int number = 0;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                mRobot.mIntake.requestState(Intake.TopLevelState.RETRACTION);
                mRobot.mDrive.followPath(scorePreload);
                setPathState(1);
                mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG);
                break;
            case 1:
                if(!mRobot.mDrive.isBusy()) {
                    mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG_SCORED);
                    if (mRobot.mLift.isComplete()) {
                        mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG_RELEASE);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!mRobot.mDrive.isBusy() && mRobot.mLift.isComplete()){
                    setPathState(3);
                }
                if (mRobot.mLift.isComplete()){
                    mRobot.mLift.request(Lift.TopLevelState.HUMAN_PLAYER_POSITION);

                }
                if(!mRobot.mDrive.isBusy()) {
                    mRobot.mDrive.followPath(firstSample);
                }
                break;
            case 3:
                mRobot.mIntake.requestState(Intake.TopLevelState.AUTO_SPECIMEN_1);
                intakeTimer.reset();
                setPathState(4);
                break;
            case 4:
                if (mRobot.mIntake.sampleInIntake() && intakeTimer.milliseconds() > 500){
                    setPathState(5);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        mRobot.mLift.update();
        mRobot.mIntake.update();

        mRobot.mDrive.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("path", number);
        telemetry.addData("x", mRobot.mDrive.getPose().getX());
        telemetry.addData("y", mRobot.mDrive.getPose().getY());
        telemetry.addData("heading", mRobot.mDrive.getPose().getHeading());


        telemetry.addData("Current Intake State", mRobot.mIntake.getCurrentIntakeState());
        telemetry.addData("Requested Intake State", mRobot.mIntake.RequestedIntakeState);

        telemetry.addData("target", mRobot.mIntake.target);
//
        telemetry.addData("Current Intake Wrist State", mRobot.mIntake.getCurrentWristState());
        telemetry.addData("Requested Intake Wrist State", mRobot.mIntake.RequstedWristState);
//
        telemetry.addData("Current Intake Power State", mRobot.mIntake.getCurrentPowerState());
        telemetry.addData("Requested Intake Power State", mRobot.mIntake.RequstedPowerState);

//        telemetry.addData("Current Intake Sample State", mRobot.mIntake.getIntakeColor());

//        telemetry.addData("Current Lift State", mRobot.mLift.getCurrentLiftState());
//        telemetry.addData("Requested Lift State", mRobot.mLift.RequestedLiftState);
//
//        telemetry.addData("Current Lift Elbow State", mRobot.mLift.getCurrentElbowState());
//        telemetry.addData("Requested Lift Elbow State", mRobot.mLift.RequestedElbowState);
//
//        telemetry.addData("Current Lift Wrist State", mRobot.mLift.getCurrentWristState());
//        telemetry.addData("Requested Lift Wrist State", mRobot.mLift.RequestedWristState);
//
//        telemetry.addData("Current Lift Claw State", mRobot.mLift.getCurrentClawState());
//        telemetry.addData("Requested Lift Claw State", mRobot.mLift.RequestedClawState);

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(mRobot.mDrive.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        mRobot.init(hardwareMap);
        mRobot.mDrive.setStartingPose(startPose);
        buildPaths();
        dashboardPoseTracker = new DashboardPoseTracker(mRobot.mDrive.poseUpdater);


        intakeList = new ArrayList<>(
                Arrays.asList(firstSample, firstTurn, secondSample, secondTurn, thirdSample, thridTurn, firstIntakeReadyPosition, firstIntakeCompletePosition, firstScore, secondIntakeReadyPosition, secondIntakeCompletePosition, secondScore, secondIntakeReadyPosition, secondIntakeCompletePosition, thirdScore, secondIntakeReadyPosition, secondIntakeCompletePosition, fourthScore, secondIntakeReadyPosition, secondIntakeCompletePosition, fifthScore, park )
        );

        scoreList = new ArrayList<>(
                Arrays.asList(secondScore, thirdScore, fourthScore, fifthScore)
        );

        mRobot.mIntake.setIntakeWristPosition(Intake.WristState.UP_POSITION.getTargetPosition());

        mRobot.mLift.setWristPosition(Lift.WristState.DEFAULT_POSITION.getTargetPosition());
        mRobot.mLift.setElbowPosition(Lift.ElbowStates.DEFAULT_POSITION.getTargetPosition());
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        mRobot.mDrive.setPose(startPose);
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        Robot.robotPose = mRobot.mDrive.getPose();
    }
}

