//package com.team9889.ftc2024.opmode;
//
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.DashboardPoseTracker;
//import com.pedropathing.util.Drawing;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import com.team9889.ftc2024.subsystems.Intake;
//import com.team9889.ftc2024.subsystems.Lift;
//import com.team9889.ftc2024.subsystems.Robot;
//
//
//@Autonomous(name = "Example Auto Blue", group = "Examples")
//public class OnePlusFour extends OpMode {
//    Robot mRobot = new Robot();
//
////    private Follower follower;
//
//    private DashboardPoseTracker dashboardPoseTracker;
//
//    private Timer pathTimer, actionTimer, opmodeTimer;
//
//
//    private int pathState;
//
//
//
//    private final Pose startPose = new Pose(7.5, 79.25, Math.toRadians(-180));
//
//    private final Pose scoreSpecimenPose = new Pose(39, 79.25, Math.toRadians(-180));
//
//    private final Pose pullBack = new Pose(35, 79.25, Math.toRadians(-180));
//
//    private final Pose turnPose = new Pose(37, 79.25, Math.toRadians(0));
//
//    private final Pose intakePose = new Pose(40, 79.25, Math.toRadians(0));
//
//    private final Pose scoreBucketPose = new Pose(16.18539976825029, 129.1494785631518, Math.toRadians(90));
//
//
//    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));
//
//    /* These are our Paths and PathChains that we will define in buildPaths() */
//    private Path scorePreload, park;
//    private PathChain pullBackPath, turnPath, intakePath, basketPath;
//
//
//    public void buildPaths() {
//
//
//        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scoreSpecimenPose)));
//        scorePreload.setTangentHeadingInterpolation();
//        scorePreload.setReversed(true);
//
//
//        pullBackPath = mRobot.mDrive.pathBuilder()
//                .addPath(new BezierLine(new Point(scoreSpecimenPose), new Point(pullBack)))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        turnPath = mRobot.mDrive.pathBuilder()
//                .addPath(new BezierLine(new Point(pullBack), new Point(turnPose)))
//                .setLinearHeadingInterpolation(pullBack.getHeading(), turnPose.getHeading())
//                .build();
//
//        intakePath = mRobot.mDrive.pathBuilder()
//                .addPath(new BezierLine(new Point(turnPose), new Point(intakePose)))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        basketPath = mRobot.mDrive.pathBuilder()
//                .addPath(new BezierCurve(
//                        new Point(40.000, 79.250, Point.CARTESIAN),
//                        new Point(6.000, 79.250, Point.CARTESIAN),
//                        new Point(29.200, 119.638, Point.CARTESIAN),
//                        new Point(16.185, 129.149, Point.CARTESIAN)
//                ))
//                .setTangentHeadingInterpolation()
//                .setReversed(true)
//                .build();
//    }
//
//    int tramsferCounter = 0;
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                mRobot.mIntake.requestState(Intake.TopLevelState.RETRACTED);
//                mRobot.mDrive.followPath(scorePreload);
//                    setPathState(1);
//                    mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG);
//                break;
//            case 1:
//                if(!mRobot.mDrive.isBusy()) {
//                    mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG_SCORED);
//                    if (mRobot.mLift.isComplete()){
//                        mRobot.mDrive.followPath(pullBackPath,true);
//                        setPathState(2);
//                        mRobot.mLift.request(Lift.TopLevelState.HIGH_RUNG_RELEASE);
//                    }
//                }
//                break;
//            case 2:
//                if(!mRobot.mDrive.isBusy()) {
//                    mRobot.mLift.request(Lift.TopLevelState.TRANSFER_PREPARE);
//
//                    mRobot.mDrive.followPath(turnPath,true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if(!mRobot.mDrive.isBusy()) {
//                    mRobot.mDrive.followPath(intakePath,true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                mRobot.mIntake.requestState(Intake.TopLevelState.AUTO_SAMPLE);
//
//                if(mRobot.mIntake.isComplete()) {
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                mRobot.mIntake.requestState(Intake.TopLevelState.RETRACTED);
//
//                if(mRobot.mIntake.isComplete()) {
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if(!mRobot.mDrive.isBusy()) {
//                    mRobot.mDrive.followPath(basketPath,true);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                switch (tramsferCounter){
//                    case 0:
//                        mRobot.mLift.request(Lift.TopLevelState.TRANSFER_READY);
//                        break;
//                    case 1:
//                        mRobot.mLift.request(Lift.TopLevelState.TRANSFER_COMPLETE);
//                        break;
//                    case 2:
//                        mRobot.mLift.request(Lift.TopLevelState.HIGH_BASKET);
//                        break;
//                    case 3:
//                        if (!mRobot.mDrive.isBusy()) {
//                            mRobot.mLift.request(Lift.TopLevelState.HIGH_BASKET_RELEASE);
//                            setPathState(8);
//                        }
//                        break;
//                }
//                if (mRobot.mLift.isComplete()){
//                    tramsferCounter += 1;
//                }
//                break;
//        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    @Override
//    public void loop() {
//
//        mRobot.mLift.update();
//        mRobot.mIntake.update();
//
//        mRobot.mDrive.update();
//        autonomousPathUpdate();
//
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", mRobot.mDrive.getPose().getX());
//        telemetry.addData("y", mRobot.mDrive.getPose().getY());
//        telemetry.addData("heading", mRobot.mDrive.getPose().getHeading());
//
//
////        telemetry.addData("Current Intake State", mRobot.mIntake.getCurrentIntakeState());
////        telemetry.addData("Requested Intake State", mRobot.mIntake.RequestedIntakeState);
////
////        telemetry.addData("Current Intake Wrist State", mRobot.mIntake.getCurrentWristState());
////        telemetry.addData("Requested Intake Wrist State", mRobot.mIntake.RequstedWristState);
////
////        telemetry.addData("Current Intake Power State", mRobot.mIntake.getCurrentPowerState());
////        telemetry.addData("Requested Intake Power State", mRobot.mIntake.RequstedPowerState);
//
////        telemetry.addData("Current Intake Sample State", mRobot.mIntake.getIntakeColor());
//
////        telemetry.addData("Current Lift State", mRobot.mLift.getCurrentLiftState());
////        telemetry.addData("Requested Lift State", mRobot.mLift.RequestedLiftState);
////
////        telemetry.addData("Current Lift Elbow State", mRobot.mLift.getCurrentElbowState());
////        telemetry.addData("Requested Lift Elbow State", mRobot.mLift.RequestedElbowState);
////
////        telemetry.addData("Current Lift Wrist State", mRobot.mLift.getCurrentWristState());
////        telemetry.addData("Requested Lift Wrist State", mRobot.mLift.RequestedWristState);
////
////        telemetry.addData("Current Lift Claw State", mRobot.mLift.getCurrentClawState());
////        telemetry.addData("Requested Lift Claw State", mRobot.mLift.RequestedClawState);
//        telemetry.update();
//
//        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
//        Drawing.drawRobot(mRobot.mDrive.getPose(), "#4CAF50");
//        Drawing.sendPacket();
//    }
//
//    /** This method is called once at the init of the OpMode. **/
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
////
////        Constants.setConstants(FConstants.class, LConstants.class);
////        follower = new Follower(hardwareMap);
//
//        mRobot.init(hardwareMap);
//        mRobot.mDrive.setStartingPose(startPose);
//        buildPaths();
//        dashboardPoseTracker = new DashboardPoseTracker(mRobot.mDrive.poseUpdater);
//
//    }
//
//    /** This method is called continuously after Init while waiting for "play". **/
//    @Override
//    public void init_loop() {}
//
//    /** This method is called once at the start of the OpMode.
//     * It runs all the setup actions, including building paths and starting the path system **/
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    /** We do not use this because everything should automatically disable **/
//    @Override
//    public void stop() {
//    }
//}
//
