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

@Autonomous(name = "TestSpecimenAuto", group = "Examples")
public class TestSpecimenAuto extends OpMode {
    Robot mRobot = new Robot();

//    private Follower follower;

    private DashboardPoseTracker dashboardPoseTracker;

    private Timer pathTimer, actionTimer, opmodeTimer;


    private int pathState;

    ElapsedTime timer = new ElapsedTime();

    private final Pose startPose = new Pose(7.000, 63.740, Math.toRadians(-180));


    double xPoint = 9.8;

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
                        new BezierLine(
                                new Point(39.212, 63.740, Point.CARTESIAN),
                                new Point(28, 37.043, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-48))
                .build();

        firstTurn = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(28, 37.043, Point.CARTESIAN),
                                new Point(31, 37.043, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(-135))
                .build();

        secondSample = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(31, 37.043, Point.CARTESIAN),
                                new Point(30, 37.043, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-65))
                .build();

        secondTurn = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(30, 37.043, Point.CARTESIAN),
                                new Point(31, 37.043, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-65), Math.toRadians(-135))
                .build();

        thirdSample = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(31, 37.043, Point.CARTESIAN),
                                new Point(30, 31, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-70))
                .build();

        thridTurn = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(30, 31, Point.CARTESIAN),
                                new Point(31, 29.034, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-70), Math.toRadians(-140))
                .build();

        firstIntakeReadyPosition = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(31, 29.034, Point.CARTESIAN),
                                new Point(15, 29.034, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(360))
                .build();

        firstIntakeCompletePosition = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(15, 29.034, Point.CARTESIAN),
                                new Point(11, 29.034, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        firstScore = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(11, 29.034, Point.CARTESIAN),
                                new Point(38.5, 68.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(180))
                .build();

        secondIntakeReadyPosition = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(38.5, 70.000, Point.CARTESIAN),
                                new Point(15, 35.900, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(360))
                .build();

        secondIntakeCompletePosition = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(15, 35.900, Point.CARTESIAN),
                                new Point(11, 35.900, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        secondScore = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(11, 29.034, Point.CARTESIAN),
                                new Point(38.5, 69.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(180))
                .build();

        thirdScore = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(11, 35.900, Point.CARTESIAN),
                                new Point(38.5, 70.00, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(180))
                .build();

        fourthScore = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 16
                        new BezierLine(
                                new Point(11, 35.900, Point.CARTESIAN),
                                new Point(38.5, 71.100, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(180))
                .build();

        fifthScore = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 19
                        new BezierLine(
                                new Point(11, 35.900, Point.CARTESIAN),
                                new Point(38.5, 72.300, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(180))
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
                if(mRobot.mDrive.getPose().getX() > 38.25) {
                    mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG_SCORED);
                    if (mRobot.mLift.isComplete()) {
                        mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG_RELEASE);
                        setPathState(2);
                        mRobot.mDrive.breakFollowing();
                    }
                }
                break;
            case 2:
                if (!mRobot.mDrive.isBusy() && mRobot.mLift.isComplete()){
                    mRobot.mLift.request(Lift.TopLevelState.TRANSFER_PREPARE);
                    setPathState(21);
                }
                break;
            case 21:
                FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.13,0);

                // driving to intake sample
                if(!mRobot.mDrive.isBusy() && mRobot.mLift.isComplete()) {
                    mRobot.mLift.request(Lift.TopLevelState.HUMAN_PLAYER_POSITION);
                    mRobot.mDrive.followPath(intakeList.get(number));
                    number += 1;
                    setPathState(3);
                }
                break;
            case 3:
                // extending when in a certain range
                double xE = mRobot.mDrive.getPose().getX() - intakeList.get(number).getPath(0).getLastControlPoint().getX();
                double yE = mRobot.mDrive.getPose().getY() - intakeList.get(number).getPath(0).getLastControlPoint().getY();

                if (!mRobot.mDrive.isBusy()) {
                    mRobot.mIntake.requestState(sampleList.get(extend));
                    extend += 1;
                    intakeTimer.reset();
                    setPathState(4);
                }
                break;
            case 4:
                // extending for second position
//                if (intakeTimer.milliseconds() > 500){
//                    mRobot.mIntake.requestState(sampleList.get(extend));
//                }

                if (mRobot.mIntake.sampleInIntake() || intakeTimer.milliseconds() > 1000) {
                    setPathState(5);
                    extend += 1;
                }
                break;
            case 5:
                // slightly retracting and turning
                if (!mRobot.mDrive.isBusy()) {
                    mRobot.mIntake.requestState(Intake.TopLevelState.AUTO_RETRACTED_2);
                    if (mRobot.mIntake.isComplete()) {
                        mRobot.mDrive.followPath(intakeList.get(number));
                        number += 1;
                        setPathState(51);
                    }
                }
                break;
            case 51:
                // changing heading PID
                if (Math.toDegrees(mRobot.mDrive.getPose().getHeading()) < 235){
//                    FollowerConstants.headingPIDFCoefficients.setCoefficients(3,0,0.13,0);
//                    mRobot.mDrive.breakFollowing();
                    outtakeTimer.reset();
                    setPathState(6);
                }
                break;
            case 6:
                // outtaking and moving either to scoring or intaking samples again
                if (outtakeTimer.milliseconds() < 250) {
                    mRobot.mIntake.requestState(Intake.TopLevelState.AUTO_OUTTAKE);
                } else {
                    if (extend < 4){
                        setPathState(21);
                        mRobot.mIntake.requestState(Intake.TopLevelState.RETRACTION);
                    } else {
                        setPathState(7);
                        mRobot.mIntake.requestState(Intake.TopLevelState.RETRACTION);
                    }
                }
                break;
            case 7:
                FollowerConstants.headingPIDFCoefficients.setCoefficients(3,0,0.13,0);
                if (!mRobot.mDrive.isBusy()){
                    mRobot.mDrive.followPath(firstIntakeReadyPosition);
                    setPathState(8);
                }
                break;
            case 8:
                if (!mRobot.mDrive.isBusy()){
                    mRobot.mDrive.followPath(firstIntakeCompletePosition);
                    setPathState(81);
                }
                break;
            case 81:
                mRobot.mDrive.setPose(new Pose(mRobot.getDistance(), mRobot.mDrive.getPose().getY(), mRobot.mDrive.getPose().getHeading()));
                if (!mRobot.mDrive.isBusy() || mRobot.mDrive.isRobotStuck()){
                    mRobot.mLift.request(Lift.TopLevelState.HUMAN_PLAYER_GRABED);
                    setPathState(82);
                }
                break;
            case 82:
                if (mRobot.mLift.isComplete()){
                    mRobot.mLift.request(Lift.TopLevelState.HUMAN_PLAYER_2);
                    setPathState(83);
                }
                break;
            case 83:
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
                    setPathState(11);
                }
                break;
            case 11:
                if (mRobot.mLift.isComplete()){
                    mRobot.mLift.request(Lift.TopLevelState.HUMAN_PLAYER_POSITION);
                    setPathState(12);
                }
                break;
            case 12:
                mRobot.mDrive.followPath(secondIntakeReadyPosition);
                setPathState(13);
                break;
            case 13:
                if (!mRobot.mDrive.isBusy()) {
                    mRobot.mDrive.followPath(secondIntakeCompletePosition);
                    setPathState(14);
                }
                break;
            case 14:
                mRobot.mDrive.setPose(new Pose(mRobot.getDistance(), mRobot.mDrive.getPose().getY(), mRobot.mDrive.getPose().getHeading()));
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
                    if (score < 4) {
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
                    if (score == 4){
                        setPathState(19);
                    } else {
                        setPathState(11);
                    }
                }
                break;
            case 19:
                if (mRobot.mLift.isComplete()) {
                    mRobot.mDrive.followPath(park);
                    mRobot.mLift.request(Lift.TopLevelState.TRANSFER_PREPARE);
                    mRobot.mIntake.requestState(Intake.TopLevelState.AUTO_OUTTAKE);
                    setPathState(20);
                }
                break;
            case 20:
                requestOpModeStop();
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
        telemetry.addData("distance", mRobot.getDistance());

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
        mRobot.mDrive.setStartingPose(startPose);
        buildPaths();
        dashboardPoseTracker = new DashboardPoseTracker(mRobot.mDrive.poseUpdater);


        intakeList = new ArrayList<>(
                Arrays.asList(firstSample, firstTurn, secondSample, secondTurn, thirdSample, thridTurn)
        );

        scoreList = new ArrayList<>(
                Arrays.asList(secondScore, thirdScore, fourthScore, fifthScore)
        );

        sampleList = new ArrayList<>(
                Arrays.asList(Intake.TopLevelState.AUTO_SPECIMEN_1, Intake.TopLevelState.AUTO_SPECIMEN_1_2, Intake.TopLevelState.AUTO_SPECIMEN_2, Intake.TopLevelState.AUTO_SPECIMEN_2_2)
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


