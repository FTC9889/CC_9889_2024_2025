package com.team9889.ftc2024.opmode;

import com.pedropathing.follower.FollowerConstants;
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
import com.qualcomm.robotcore.util.RobotLog;
import com.team9889.ftc2024.subsystems.Intake;
import com.team9889.ftc2024.subsystems.Lift;
import com.team9889.ftc2024.subsystems.Robot;
import com.team9889.lib.pedroPathing.constants.FConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.MissingResourceException;

@Autonomous(name = "SpecimenAuto", group = "Examples", preselectTeleOp = "TeleOperate")
public class SpecimenAuto extends OpMode {
    Robot mRobot = new Robot();

//    private Follower follower;

    private DashboardPoseTracker dashboardPoseTracker;

    private Timer pathTimer, actionTimer, opmodeTimer;


    private int pathState;

    ElapsedTime timer = new ElapsedTime();

    private final Pose startPose = new Pose(7.000, 63.740, Math.toRadians(-180));


    double xPoint = 9.8;

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreload, firstScore, secondIntakeCompletePosition, secondScore, thirdScore, fourthScore, secondIntakeReadyPosition, park, firstSamplePush, secondSamplePush;


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

        firstSamplePush = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(39.212, 63.740, Point.CARTESIAN),
                                new Point(15.685, 64.575, Point.CARTESIAN),
                                new Point(34.039, 30.035, Point.CARTESIAN),
                                new Point(59.402, 31.537, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(59.402, 31.537, Point.CARTESIAN),
                                new Point(63.407, 20.023, Point.CARTESIAN),
                                new Point(14.684, 23.527, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        secondSamplePush = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(14.684, 23.527, Point.CARTESIAN),
                                new Point(72.250, 26.030, Point.CARTESIAN),
                                new Point(55.564, 12.00, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))

                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(55.564, 15.020, Point.CARTESIAN),
                                new Point(8.0, 19.020, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setReversed(true)
                .build();

        firstScore = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(8, 17.020, Point.CARTESIAN),
                                new Point(38.5, 68.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(180))
                .build();

        secondIntakeReadyPosition = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(38, 68.000, Point.CARTESIAN),
                                new Point(14, 35.900, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(0))
                .build();

        secondIntakeCompletePosition = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(14, 35.900, Point.CARTESIAN),
                                new Point(9.38, 35.900, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        secondScore = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(9.3, 29.034, Point.CARTESIAN),
                                new Point(38, 69.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        thirdScore = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(9.3, 35.900, Point.CARTESIAN),
                                new Point(39, 70.00, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        park = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 20
                        new BezierLine(
                                new Point(38, 70.00, Point.CARTESIAN),
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
    ElapsedTime intakeTimer2 = new ElapsedTime();
    ElapsedTime outtakeTimer = new ElapsedTime();


    boolean up = true;

    int number = 0;
    int extend = 0;
    int score = 0;



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                mRobot.mIntake.requestState(Intake.TopLevelState.RETRACTION);
                mRobot.mDrive.followPath(scorePreload);
                setPathState(1);
                mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG);
                break;
            case 1:
                if (mRobot.mDrive.getPose().getX() > 38.25) {
                    mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG_SCORED);
                    if (mRobot.mLift.isComplete()) {
                        mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG_RELEASE);
                        setPathState(2);
                        mRobot.mDrive.breakFollowing();
                    }
                }
                break;
            case 2:
                if (!mRobot.mDrive.isBusy() && mRobot.mLift.isComplete()) {
                    mRobot.mLift.request(Lift.TopLevelState.TRANSFER_PREPARE);
                    setPathState(3);
                }
                break;
            case 3:
                // driving to intake sample
                if (!mRobot.mDrive.isBusy() || (number == 1 && mRobot.mDrive.getPose().getX() < 18)){
                    mRobot.mLift.request(Lift.TopLevelState.HUMAN_PLAYER_POSITION);
                    mRobot.mDrive.followPath(intakeList.get(number));
                    number += 1;
                    setPathState(5);
                }
                break;
            case 5:
                if (number < 2){
                    setPathState(3);
                } else {
                    setPathState(6);
                }

                break;
            case 6:
                if (!mRobot.mDrive.isBusy()){
                    mRobot.mLift.request(Lift.TopLevelState.HUMAN_PLAYER_GRABED);
                    setPathState(7);
                }
                break;
            case 7:
                if (mRobot.mLift.isComplete()){
                    mRobot.mLift.request(Lift.TopLevelState.HUMAN_PLAYER_2);
                    setPathState(8);
                }
                break;
            case 8:
                if (mRobot.mLift.isComplete()){
                    mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG);
                    mRobot.mDrive.followPath(firstScore);
                    setPathState(9);
                }
                break;
            case 9:
                if (!mRobot.mDrive.isBusy() && mRobot.mLift.isComplete()){
                    mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG_SCORED);
                    setPathState(10);
                }
                break;
            case 10:
                if (mRobot.mLift.isComplete()){
                    mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG_RELEASE);
                    setPathState(13);
                }
                break;
            case 13:
                if (!mRobot.mDrive.isBusy() && mRobot.mLift.isComplete()){
                    mRobot.mDrive.followPath(secondIntakeReadyPosition);
                    mRobot.mLift.request(Lift.TopLevelState.HUMAN_PLAYER_POSITION);
                    setPathState(131);
                }
            case 131:
                if (!mRobot.mDrive.isBusy() && mRobot.mLift.isComplete()) {
                    mRobot.mDrive.followPath(secondIntakeCompletePosition);
                    setPathState(14);
                }
                break;
            case 14:
                if (!mRobot.mDrive.isBusy()){
                    mRobot.mLift.request(Lift.TopLevelState.HUMAN_PLAYER_GRABED);
                    setPathState(15);
                }
                break;
            case 15:
                if (mRobot.mLift.isComplete()){
                    mRobot.mLift.request(Lift.TopLevelState.HUMAN_PLAYER_2);
                    setPathState(16);
                }
                break;
            case 16:
                if (mRobot.mLift.isComplete()){
                    mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG);
                    mRobot.mDrive.followPath(scoreList.get(score));
                    if (score < 2) {
                        score += 1;
                    }
                    setPathState(17);
                }
                break;
            case 17:
                if (!mRobot.mDrive.isBusy() && mRobot.mLift.isComplete()){
                    mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG_SCORED);
                    setPathState(18);
                }
                break;
            case 18:
                if (mRobot.mLift.isComplete()){
                    mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG_RELEASE);
                    if (score < 2){
                        setPathState(13);
                    } else {
                        setPathState(19);
                    }
                }
                break;
            case 19:
                if (mRobot.mLift.isComplete()) {
                    mRobot.mDrive.followPath(park);
                    mRobot.mLift.request(Lift.TopLevelState.TRANSFER_PREPARE);
                    mRobot.mIntake.requestState(Intake.TopLevelState.AUTO_PARK_EXTEND);
                    setPathState(20);
                }
                break;
            case 20:
                if (!mRobot.mDrive.isBusy()) {
                    requestOpModeStop();
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

//        if () {
//            requestOpModeStop();
//        }
        RobotLog.d("Time: " + opmodeTimer.getElapsedTimeSeconds() +
//                " | Path state: ", pathState +
                " | Is Nan: " +  mRobot.mDrive.isLocalizationNAN() +
                " | X: " + mRobot.mDrive.getPose().getX() +
                " | Y: " + mRobot.mDrive.getPose().getY() +
                " | T: " + Math.toDegrees(mRobot.mDrive.getPose().getHeading()) +
                " | Pro: " + FollowerConstants.headingPIDFCoefficients.P);

//        telemetry.addData("NAN", mRobot.mDrive.isLocalizationNAN());
        telemetry.addData("extend", extend);
        telemetry.addData("intakeNumber", number);

        telemetry.addData("path state", pathState);
        telemetry.addData("path", number);
        telemetry.addData("x", mRobot.mDrive.getPose().getX());
        telemetry.addData("y", mRobot.mDrive.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(mRobot.mDrive.getPose().getHeading()));


        telemetry.addData("Current Intake State", mRobot.mIntake.getCurrentIntakeState());
        telemetry.addData("Requested Intake State", mRobot.mIntake.RequestedIntakeState);

        telemetry.addData("target", mRobot.mIntake.target);
//
        telemetry.addData("Current Intake Wrist State", mRobot.mIntake.getCurrentWristState());
        telemetry.addData("Requested Intake Wrist State", mRobot.mIntake.RequstedWristState);
//
        telemetry.addData("Current Intake Power State", mRobot.mIntake.getCurrentPowerState());
        telemetry.addData("Requested Intake Power State", mRobot.mIntake.RequstedPowerState);

        telemetry.addData("P", FollowerConstants.headingPIDFCoefficients.P);

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
        mRobot.mIntake.auto = true;
        mRobot.mDrive.setStartingPose(startPose);
        buildPaths();
        dashboardPoseTracker = new DashboardPoseTracker(mRobot.mDrive.poseUpdater);


//        intakeList = new ArrayList<>(
//                Arrays.asList(firstSample, firstTurn, secondSample, secondTurn, thirdSample, thridTurn)
//        );

        intakeList = new ArrayList<>(
                Arrays.asList(firstSamplePush, secondSamplePush)
//                        , thirdSample, thridTurn)
        );


        scoreList = new ArrayList<>(
                Arrays.asList(secondScore, thirdScore)
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
        mRobot.mLift.setClawPosition(0.7);
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        Robot.robotPose = mRobot.mDrive.getPose();
    }
}

