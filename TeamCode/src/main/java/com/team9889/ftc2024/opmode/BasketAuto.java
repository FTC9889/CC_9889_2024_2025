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


@Autonomous(name = "BasketAuto", group = "Examples", preselectTeleOp = "TeleOperate")
public class BasketAuto extends OpMode {
    Robot mRobot = new Robot();

//    private Follower follower;

    private DashboardPoseTracker dashboardPoseTracker;

    private Timer pathTimer, actionTimer, opmodeTimer;


    private int pathState;

    int count = 0;
    double value = 0;

    ElapsedTime timer = new ElapsedTime();

    private final Pose startPose = new Pose(7.84, 103.45, Math.toRadians(-90));

    private final Pose scorePoint = new Pose(15, 128, Math.toRadians(-45));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreload;
    private PathChain secondSample, secondSample2, scoreSecondSample, thirdSample, thirdSample2, scoreThirdSample, fourthSample, fourthSample2, scoreFourthSample, park;


    public void buildPaths() {


        scorePreload = mRobot.mDrive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),
                                new Point(14, 130, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .build();


        secondSample = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(14, 130, Point.CARTESIAN),
                                new Point(36, 120.81, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        secondSample2 = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(36, 120.81, Point.CARTESIAN),
                                new Point(32, 120.81, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        scoreSecondSample = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                secondSample.getPath(0).getLastControlPoint(),
                                new Point(scorePoint.getX(), scorePoint.getY(), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-12), scorePoint.getHeading())
                .build();

        thirdSample = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                scoreSecondSample.getPath(0).getLastControlPoint(),
                                new Point(36, 131.65, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        thirdSample2 = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(36, 131.65, Point.CARTESIAN),
                                new Point(33, 131.65, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        scoreThirdSample = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                thirdSample.getPath(0).getLastControlPoint(),
                                new Point(scorePoint.getX(), scorePoint.getY(), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), scorePoint.getHeading())
                .build();

        fourthSample = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(scorePoint.getX(), scorePoint.getY(), Point.CARTESIAN),
                                new Point(40, 132, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(50))
                .build();

        fourthSample2 = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(40, 132, Point.CARTESIAN),
                                new Point(45.22, 125, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(90))
                .build();

        scoreFourthSample = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(45.22, 125, Point.CARTESIAN),
                                new Point(scorePoint.getX(), scorePoint.getY(), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), scorePoint.getHeading())
                .build();

        park = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(14.000, 128.000, Point.CARTESIAN),
                                new Point(64.500, 118.000, Point.CARTESIAN),
                                new Point(65.300, 95.400, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .build();
    }


    int sampleNumber = 0;
    int pickupNumber = 0;
    public ArrayList<PathChain> scoreList;

    public ArrayList<PathChain> pickupList;

    public ArrayList<Intake.TopLevelState> sampleList ;

    ElapsedTime scoreTimer = new ElapsedTime();
    ElapsedTime anotherTimer = new ElapsedTime();
    ElapsedTime retractionTimer = new ElapsedTime();
    ElapsedTime otherOtherTimer = new ElapsedTime();
    ElapsedTime otherOtherOtherTimer = new ElapsedTime();


    boolean up = true;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                mRobot.mIntake.requestState(Intake.TopLevelState.RETRACTION);
                mRobot.mDrive.followPath(scorePreload);
                setPathState(1);
                mRobot.mLift.request(Lift.TopLevelState.HIGH_BASKET);
                mRobot.mIntake.requestState(Intake.TopLevelState.OUTTAKE);
                break;
            case 1:
                if (!mRobot.mDrive.isBusy()) {
                    mRobot.mLift.request(Lift.TopLevelState.HIGH_BASKET_READY);
                    setPathState(111);
                }
                break;
            case 111:
                setPathState(11);
                timer.reset();
                break;
            case 11:
                if (mRobot.mLift.isComplete()) {
                    if (timer.milliseconds() > 500) {
                        mRobot.mLift.request(Lift.TopLevelState.HIGH_BASKET_RELEASE);
                        mRobot.mIntake.requestState(Intake.TopLevelState.AUTO_RETRACTED);
                        setPathState(1111);
                    }
                } else {
                    timer.reset();
                }
                break;
            case 1111:
                if (sampleNumber == 3) {
                    if (mRobot.mLift.isComplete()) {
                        setPathState(1000);
                        mRobot.mDrive.followPath(park, true);
                    }
                } else {
                    setPathState(112);
                }
                break;
            case 112:
                if (mRobot.mLift.isComplete()) {
                    mRobot.mIntake.requestState(sampleList.get(sampleNumber));
                    mRobot.mDrive.followPath(pickupList.get(pickupNumber), true);
                    setPathState(113);
                }
                break;
            case 113:
                if (mRobot.mDrive.getPose().getX() > 20) {
                    mRobot.mLift.request(Lift.TopLevelState.TRANSFER_PREPARE);
                    timer.reset();
                    if (pickupNumber < 6) {
                        setPathState(114);
                        pickupNumber += 1;
                    }
                }
                break;
            case 114:
                if (mRobot.mIntake.sampleInIntake()) {
                    setPathState(4);
                }
                if (!mRobot.mDrive.isBusy() && mRobot.mIntake.isComplete()) {
                    mRobot.mDrive.followPath(pickupList.get(pickupNumber), true);
                    setPathState(3);
                    timer.reset();
                }
                break;
            case 3:
                if (mRobot.mIntake.sampleInIntake() || timer.milliseconds() > 500) {
                    setPathState(4);
                }
                break;
            case 4:
                if (!mRobot.mDrive.isBusy()) {
                    mRobot.mDrive.followPath(scoreList.get(sampleNumber));
                    retractionTimer.reset();
                    setPathState(5);
                }
                break;
            case 5:
                if (mRobot.mIntake.isComplete() || mRobot.mIntake.sampleInIntake()) {
                    mRobot.mIntake.requestState(Intake.TopLevelState.AUTO_RETRACTED);
                    setPathState(51);
                    timer.reset();
                }
                break;
            case 51:
                if (mRobot.mIntake.isComplete()) {
                    mRobot.mIntake.setFlickerPosition(0.83);
                    mRobot.mLift.request(Lift.TopLevelState.TRANSFER_READY);
                    anotherTimer.reset();
                    setPathState(6);
                }
                break;
            case 6:
                if (mRobot.mLift.isComplete() && mRobot.mIntake.isComplete()) {
                    if (anotherTimer.milliseconds() > 200) {
                        mRobot.mLift.request(Lift.TopLevelState.TRANSFER_COMPLETE);
                        timer.reset();
                        otherOtherTimer.reset();
                        setPathState(61);
                    }
                } else {
                    anotherTimer.reset();
                }
                break;
            case 61:
                if (mRobot.mLift.isComplete()) {
                    if (otherOtherTimer.milliseconds() > 200) {
                        mRobot.mLift.request(Lift.TopLevelState.SCORE_PREPARE);
                        timer.reset();
                        setPathState(7);
                    }
                } else {
                    otherOtherTimer.reset();
                }
                break;
            case 62:
                if (mRobot.mIntake.sampleInIntake()) {
                    value += 1;
                } else {
                    value -= 1;
                }

                count++;

                if (count > 10) {
                    if (value > 0) {
                        setPathState(5);
                        value = 0;
                    } else {
                        otherOtherOtherTimer.reset();
                        setPathState(7);
                        value = 0;
                    }
                }
                break;
            case 7:
                if(mRobot.mLift.isComplete() && !mRobot.mDrive.isBusy()) {
                    mRobot.mIntake.setFlickerPosition(0.34);
                    if (otherOtherOtherTimer.milliseconds() > 500) {
                        sampleNumber += 1;
                        pickupNumber += 1;
                        mRobot.mLift.request(Lift.TopLevelState.HIGH_BASKET);
                        mRobot.mIntake.requestState(Intake.TopLevelState.AUTO_OUTTAKE);
                        timer.reset();
                        setPathState(1);
                    }
                }

//                if (mRobot.mIntake.sampleInIntake() && timer.milliseconds() > 700){
//                    mRobot.mLift.request(Lift.TopLevelState.TRANSFER_PREPARE);
//                    setPathState(500);
//                } else {
//                    mRobot.mLift.request(Lift.TopLevelState.HIGH_BASKET_READY);
//                }
                break;

            case 1000:
                if (mRobot.mDrive.getPose().getX() > 20) {
                    mRobot.mLift.request(Lift.TopLevelState.TRANSFER_PREPARE);
                    setPathState(1001);
                    anotherTimer.reset();
                }
                break;
            case 1001:
//                if (!mRobot.mDrive.isBusy()) {
                    mRobot.mFlag.setFlagPosition(0.5);
//                }
                if (!mRobot.mDrive.isBusy() && mRobot.mLift.isComplete() && anotherTimer.milliseconds() > 3000){
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

        telemetry.addData("path state", pathState);
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
        telemetry.update();

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
//
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);

        mRobot.init(hardwareMap);
        mRobot.mIntake.auto = false;
        mRobot.mDrive.setStartingPose(startPose);
        buildPaths();
        dashboardPoseTracker = new DashboardPoseTracker(mRobot.mDrive.poseUpdater);

        scoreList = new ArrayList<>(
                Arrays.asList(scoreSecondSample, scoreThirdSample, scoreFourthSample)
        );

        pickupList = new ArrayList<>(
                Arrays.asList(secondSample, secondSample2, thirdSample, thirdSample2,fourthSample, fourthSample2)
        );

        sampleList = new ArrayList<Intake.TopLevelState>(
                Arrays.asList(Intake.TopLevelState.AUTO_SAMPLE, Intake.TopLevelState.AUTO_SAMPLE, Intake.TopLevelState.AUTO_SAMPLE)
        );

        mRobot.mIntake.setIntakeWristPosition(Intake.WristState.UP_POSITION.getTargetPosition());

        mRobot.mLift.setWristPosition(Lift.WristState.DEFAULT_POSITION.getTargetPosition());
        mRobot.mLift.setElbowPosition(Lift.ElbowStates.DEFAULT_POSITION.getTargetPosition());

        mRobot.mFlag.setFlagPosition(0.9);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        Robot.robotPose = mRobot.mDrive.getPose();
    }
}

