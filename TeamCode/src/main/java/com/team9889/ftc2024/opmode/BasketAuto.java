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


@Autonomous(name = "BasketAuto", group = "Examples")
public class BasketAuto extends OpMode {
    Robot mRobot = new Robot();

//    private Follower follower;

    private DashboardPoseTracker dashboardPoseTracker;

    private Timer pathTimer, actionTimer, opmodeTimer;


    private int pathState;

    ElapsedTime timer = new ElapsedTime();

    private final Pose startPose = new Pose(7.84, 103.45, Math.toRadians(-90));

    private final Pose scorePoint = new Pose(13.01, 129.65, Math.toRadians(-45));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreload;
    private PathChain secondSample, scoreSecondSample, thirdSample, scoreThirdSample, fourthSample, scoreFourthSample;


    public void buildPaths() {


        scorePreload = mRobot.mDrive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),
                                new Point(30.202, 106.290, Point.CARTESIAN),
                                new Point(22.526, 119.138, Point.CARTESIAN),
                                new Point(scorePoint.getX(), scorePoint.getY(), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePoint.getHeading())
                .setReversed(true)
            .build();


        secondSample = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                scorePreload.getPath(0).getLastControlPoint(),
                                new Point(37.71, 120.81, Point.CARTESIAN)
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
                                new Point(36.71, 131.65, Point.CARTESIAN)
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
                                new Point(45.22, 132, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90))
                .build();

        scoreFourthSample = mRobot.mDrive.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                fourthSample.getPath(0).getLastControlPoint(),
                                new Point(scorePoint.getX(), scorePoint.getY(), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), scorePoint.getHeading())
                .build();

    }


    int sampleNumber = 0;
    public ArrayList<PathChain> scoreList;

    public ArrayList<PathChain> pickupList;

    public ArrayList<Intake.TopLevelState> sampleList ;


    boolean up = true;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                mRobot.mIntake.requestState(Intake.TopLevelState.RETRACTION);
                mRobot.mDrive.followPath(scorePreload);
                setPathState(1);
                mRobot.mLift.request(Lift.TopLevelState.HIGH_BASKET);
                break;
            case 1:
                if(!mRobot.mDrive.isBusy()) {
                    mRobot.mLift.request(Lift.TopLevelState.HIGH_BASKET_READY);
                    setPathState(11);
                }

                up = true;
                break;
            case 11:
                if (mRobot.mLift.isComplete()){
                    mRobot.mLift.request(Lift.TopLevelState.HIGH_BASKET_RELEASE);
                }
                if (mRobot.mLift.isComplete()){
                    mRobot.mIntake.requestState(sampleList.get(sampleNumber));
                    mRobot.mDrive.followPath(pickupList.get(sampleNumber),true);
                    setPathState(2);
                }
                break;
            case 2:

                if(mRobot.mDrive.getPose().getX() > 19) {
                    mRobot.mLift.request(Lift.TopLevelState.TRANSFER_PREPARE);
                    setPathState(3);
                    timer.reset();
                }
                break;
            case 3:

                if (mRobot.mIntake.sampleInIntake()){
                    setPathState(4);
                } else if (timer.milliseconds() > 1000){
                    setPathState(4);
                }

                break;
            case 4:
                if(!mRobot.mDrive.isBusy()) {
                        mRobot.mIntake.requestState(Intake.TopLevelState.AUTO_RETRACTED);
                        mRobot.mDrive.followPath(scoreList.get(sampleNumber),true);

                        setPathState(5);

                }
                break;
            case 5:
                if (mRobot.mIntake.isComplete()){
                    mRobot.mLift.request(Lift.TopLevelState.TRANSFER_READY);
                    setPathState(6);
                    timer.reset();
                }
                break;
            case 6:
                if (mRobot.mLift.isComplete()) {
                    if (timer.milliseconds() > 500) {
                        setPathState(7);
                        timer.reset();
                    }
                    mRobot.mLift.request(Lift.TopLevelState.TRANSFER_COMPLETE);
                }
                break;
            case 7:
                if(mRobot.mLift.isComplete() && !mRobot.mDrive.isBusy()) {
                    if(up) {
                        sampleNumber += 1;
                        up = false;
                    }

                    if (timer.milliseconds() > 500){
                        mRobot.mLift.request(Lift.TopLevelState.HIGH_BASKET);
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
        mRobot.mDrive.setStartingPose(startPose);
        buildPaths();
        dashboardPoseTracker = new DashboardPoseTracker(mRobot.mDrive.poseUpdater);

        scoreList = new ArrayList<>(
                Arrays.asList(scoreSecondSample, scoreThirdSample, scoreFourthSample)
        );

        pickupList = new ArrayList<>(
                Arrays.asList(secondSample, thirdSample, fourthSample)
        );

        sampleList = new ArrayList<Intake.TopLevelState>(
                Arrays.asList(Intake.TopLevelState.AUTO_SAMPLE, Intake.TopLevelState.AUTO_SAMPLE, Intake.TopLevelState.AUTO_SAMPLE)
        );

        mRobot.mIntake.setIntakeWristPosition(Intake.WristState.UP_POSITION.getTargetPosition());

        mRobot.mLift.setWristPosition(Lift.WristState.INTAKE_POSITION.getTargetPosition());
        mRobot.mLift.setElbowPosition(Lift.ElbowStates.INTAKE_POSITION.getTargetPosition());
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

