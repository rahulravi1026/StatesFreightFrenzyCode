package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.AyeshaCancelable;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.webcam.TSEPositionBluePipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is an example of a more complex path to really test the tuning.
 */

/**
 Forward is positive x
 Backward is negative x
 Right is negative y
 Left is positive y
 */

@Autonomous(group = "drive", preselectTeleOp = "BlueTeleOp")
public class AutoBlue extends LinearOpMode {

    AyeshaCancelable ayesha;

    OpenCvCamera webcam;
    TSEPositionBluePipeline pipeline;

    boolean cycle1Collected, cycle2Collected, cycle3Collected, cycle4Collected, cycle5Collected = false;

    TrajectorySequence exitWarehouseCycle1, exitWarehouseCycle2, exitWarehouseCycle3, exitWarehouseCycle4, exitWarehouseCycle5, exitWarehouseCycle6, exitWarehouseCycle7;
    Trajectory parkInWarehouse;

    int jiggleCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ayesha = new AyeshaCancelable(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        ayesha.setPoseEstimate(startPose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "backWebcam"), cameraMonitorViewId);

        pipeline = new TSEPositionBluePipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) { }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        Thread liftDropperPreloaded1 = new LiftDropper(1);
        Thread liftDropperPreloaded2 = new LiftDropper(2);
        Thread liftDropperPreloaded3 = new LiftDropper(3);

        Thread readyForCollection = new ReadyForCollection();
        Thread readyForCollectionNoStart = new ReadyForCollectionNoStart();
        Thread completeCollection = new CompleteCollection();

        Thread scoreFreightCycle = new ScoreFreightCycle();
        Thread scoreFreightPreloaded = new ScoreFreightPreloaded();

        Thread rotateCapX = new RotateCapX();
        Thread spinMeasuringTape = new SpinMeasuringTape();

        Trajectory placePreloadedShippingHubLevel3Or2 = ayesha.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-8.5, -16.5, Math.toRadians(70)))
                .addDisplacementMarker(0.9, 0, () -> {
                    scoreFreightPreloaded.start();
                })
                .build();

        Trajectory placePreloadedShippingHubLevel1 = ayesha.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-9, -23.5, Math.toRadians(70))) //=8.5
                .addDisplacementMarker(0.9, 0, () -> {
                    scoreFreightPreloaded.start();
                })
                .build();

        Trajectory enterWarehouseCycle1Level3Or2 = ayesha.trajectoryBuilder(placePreloadedShippingHubLevel3Or2.end())
                .splineToSplineHeading(new Pose2d(26, 1.5, Math.toRadians(0)), Math.toRadians(0))
                .splineTo(new Vector2d(50, 1.5), Math.toRadians(0),
                        AyeshaCancelable.getVelocityConstraint(27, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        AyeshaCancelable.getAccelerationConstraint(27))
                .addDisplacementMarker(0.18, 0, () -> {
                    ayesha.lowerDropper();
                    readyForCollection.start();
                })
                .build();

        Trajectory enterWarehouseCycle1Level1 = ayesha.trajectoryBuilder(placePreloadedShippingHubLevel1.end())
                .splineToSplineHeading(new Pose2d(26, 1.5, Math.toRadians(0)), Math.toRadians(0))
                .splineTo(new Vector2d(50, 1.5), Math.toRadians(0),
                        AyeshaCancelable.getVelocityConstraint(27, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        AyeshaCancelable.getAccelerationConstraint(27))
                .addDisplacementMarker(0.18, 0, () -> {
                    ayesha.lowerDropper();
                    readyForCollection.start();
                })
                .build();

        waitForStart();

        if (isStopRequested()) return;

        if(pipeline.getAnalysis() == TSEPositionBluePipeline.TSEPosition.RIGHT) { //Level 3
            liftDropperPreloaded3.start();
            ayesha.followTrajectory(placePreloadedShippingHubLevel3Or2);
            ayesha.followTrajectoryAsync(enterWarehouseCycle1Level3Or2);
        }
        else if(pipeline.getAnalysis() == TSEPositionBluePipeline.TSEPosition.CENTER) { //Level 2
            liftDropperPreloaded2.start();
            ayesha.followTrajectory(placePreloadedShippingHubLevel3Or2);
            ayesha.followTrajectoryAsync(enterWarehouseCycle1Level3Or2);
        }
        else if(pipeline.getAnalysis() == TSEPositionBluePipeline.TSEPosition.LEFT) { //Level 1
            liftDropperPreloaded1.start();
            ayesha.followTrajectory(placePreloadedShippingHubLevel1);
            ayesha.followTrajectoryAsync(enterWarehouseCycle1Level1);
        }

        while(opModeIsActive() && ayesha.isBusy()) {
            if(ayesha.retrieveCollectorDistance() < 3.3) {
                cycle1Collected = true;
                ayesha.breakFollowing();
                ayesha.setDrivePower(new Pose2d());

                exitWarehouseCycle1 = ayesha.trajectorySequenceBuilder(new Pose2d(ayesha.getPoseEstimate().getX(), 1.5, Math.toRadians(0)))
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(-8.5, -16.8, Math.toRadians(70)), Math.toRadians(-110))
                        .build();

                completeCollection.start();
                ayesha.followTrajectorySequence(exitWarehouseCycle1);
                scoreFreightPreloaded.start();

                break;
            }
            ayesha.update();
        }

        if(!cycle1Collected) {
            jiggleCount++;

            ayesha.turn(Math.toRadians(-11));
            cycle1Collected = true;

            exitWarehouseCycle1 = ayesha.trajectorySequenceBuilder(new Pose2d(ayesha.getPoseEstimate().getX(), 1.5, Math.toRadians(-11)))
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(-8.5, -16.8, Math.toRadians(70)), Math.toRadians(-110))
                    .build();

            while(opModeIsActive() && exitWarehouseCycle1 == null) {}

            completeCollection.start();
            ayesha.followTrajectorySequence(exitWarehouseCycle1);
            scoreFreightPreloaded.start();
        }

        Trajectory enterWarehouseCycle2 = ayesha.trajectoryBuilder(exitWarehouseCycle1.end())
                .splineToSplineHeading(new Pose2d(30, 1.5, Math.toRadians(0)), Math.toRadians(0))
                .splineTo(new Vector2d(52, 1.5), Math.toRadians(0),
                        AyeshaCancelable.getVelocityConstraint(27, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        AyeshaCancelable.getAccelerationConstraint(27))
                .addDisplacementMarker(0.18, 0, () -> {
                    ayesha.lowerDropper();
                    readyForCollection.start();
                })
                .build();

        ayesha.followTrajectoryAsync(enterWarehouseCycle2);

        while(opModeIsActive() && ayesha.isBusy()) {
            if(ayesha.retrieveCollectorDistance() < 3.3) {
                cycle2Collected = true;
                ayesha.breakFollowing();
                ayesha.setDrivePower(new Pose2d());

                exitWarehouseCycle2 = ayesha.trajectorySequenceBuilder(new Pose2d(ayesha.getPoseEstimate().getX(), 1.5, Math.toRadians(0)))
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(-7.4, -16.8, Math.toRadians(70)), Math.toRadians(-110))
                        .build();

                completeCollection.start();
                ayesha.followTrajectorySequence(exitWarehouseCycle2);
                scoreFreightPreloaded.start();

                break;
            }
            ayesha.update();
        }

        if(!cycle2Collected) {
            jiggleCount++;

            ayesha.turn(Math.toRadians(-11));
            cycle2Collected = true;

            exitWarehouseCycle2 = ayesha.trajectorySequenceBuilder(new Pose2d(ayesha.getPoseEstimate().getX(), 1.5, Math.toRadians(-11)))
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(-7.4, -16.8, Math.toRadians(70)), Math.toRadians(-110))
                    .build();

            while(opModeIsActive() && exitWarehouseCycle2 == null) {}

            completeCollection.start();
            ayesha.followTrajectorySequence(exitWarehouseCycle2);
            scoreFreightPreloaded.start();
        }

        Trajectory enterWarehouseCycle3 = ayesha.trajectoryBuilder(exitWarehouseCycle2.end())
                .splineToSplineHeading(new Pose2d(33, 2.2, Math.toRadians(0)), Math.toRadians(0))
                .splineTo(new Vector2d(55, 2.2), Math.toRadians(0),
                        AyeshaCancelable.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        AyeshaCancelable.getAccelerationConstraint(30))
                .addDisplacementMarker(0.18, 0, () -> {
                    ayesha.lowerDropper();
                    readyForCollectionNoStart.start();
                })
                .addDisplacementMarker(0.65, 0, () -> {
                    ayesha.lowerCollector();
                    ayesha.startCollector();
                })
                .build();

        ayesha.followTrajectoryAsync(enterWarehouseCycle3);

        while(opModeIsActive() && ayesha.isBusy()) {
            if(ayesha.retrieveCollectorDistance() < 3.3) {
                cycle3Collected = true;
                ayesha.breakFollowing();
                ayesha.setDrivePower(new Pose2d());

                exitWarehouseCycle3 = ayesha.trajectorySequenceBuilder(new Pose2d(ayesha.getPoseEstimate().getX(), 2.2, Math.toRadians(0)))
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(-6, -18.3, Math.toRadians(70)), Math.toRadians(-110))
                        .build();

                completeCollection.start();
                ayesha.followTrajectorySequence(exitWarehouseCycle3);
                scoreFreightPreloaded.start();

                break;
            }
            ayesha.update();
        }

        if(!cycle3Collected) {
            jiggleCount++;

            ayesha.turn(Math.toRadians(-11));
            cycle3Collected = true;

            exitWarehouseCycle3 = ayesha.trajectorySequenceBuilder(new Pose2d(ayesha.getPoseEstimate().getX(), 2.2, Math.toRadians(-11)))
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(-6.5, -18.3, Math.toRadians(70)), Math.toRadians(-110))
                    .build();

            while(opModeIsActive() && exitWarehouseCycle3 == null) {}

            completeCollection.start();
            ayesha.followTrajectorySequence(exitWarehouseCycle3);
            scoreFreightPreloaded.start();
        }

        Trajectory enterWarehouseCycle4 = ayesha.trajectoryBuilder(exitWarehouseCycle3.end())
                .splineToSplineHeading(new Pose2d(35.3, 3, Math.toRadians(0)), Math.toRadians(0))
                .splineTo(new Vector2d(56, 3), Math.toRadians(0),
                        AyeshaCancelable.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        AyeshaCancelable.getAccelerationConstraint(30))
                .addDisplacementMarker(0.18, 0, () -> {
                    ayesha.lowerDropper();
                    readyForCollectionNoStart.start();
                })
                .addDisplacementMarker(0.75, 0, () -> {
                    ayesha.lowerCollector();
                    ayesha.startCollector();
                })
                .build();

        ayesha.followTrajectoryAsync(enterWarehouseCycle4);

        while(opModeIsActive() && ayesha.isBusy()) {
            if(ayesha.retrieveCollectorDistance() < 3.3) {
                cycle4Collected = true;
                ayesha.breakFollowing();
                ayesha.setDrivePower(new Pose2d());

                exitWarehouseCycle4 = ayesha.trajectorySequenceBuilder(new Pose2d(ayesha.getPoseEstimate().getX(), 3, Math.toRadians(0)))
                        .setReversed(true)
                        //.splineTo(new Vector2d(25, 3), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-4, -20, Math.toRadians(70)), Math.toRadians(-110))
                        .build();

                completeCollection.start();
                ayesha.followTrajectorySequence(exitWarehouseCycle4);
                scoreFreightPreloaded.start();

                break;
            }
            ayesha.update();
        }

        if(!cycle4Collected) {
            jiggleCount++;

            ayesha.turn(Math.toRadians(-11));
            cycle4Collected = true;

            exitWarehouseCycle4 = ayesha.trajectorySequenceBuilder(new Pose2d(ayesha.getPoseEstimate().getX(), 3, Math.toRadians(-11)))
                    .setReversed(true)
                    .splineTo(new Vector2d(25, 3), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-4, -20, Math.toRadians(70)), Math.toRadians(-110))
                    .build();

            while(opModeIsActive() && exitWarehouseCycle4 == null) {}

            completeCollection.start();
            ayesha.followTrajectorySequence(exitWarehouseCycle4);
            scoreFreightPreloaded.start();
        }

        Trajectory enterWarehouseCycle5 = ayesha.trajectoryBuilder(exitWarehouseCycle4.end())
                .splineToSplineHeading(new Pose2d(36.5, 3, Math.toRadians(0)), Math.toRadians(0))
                .splineTo(new Vector2d(58, 3), Math.toRadians(0),
                        AyeshaCancelable.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        AyeshaCancelable.getAccelerationConstraint(30))
                .addDisplacementMarker(0.18, 0, () -> {
                    ayesha.lowerDropper();
                    readyForCollectionNoStart.start();
                })
                .addDisplacementMarker(0.75, 0, () -> {
                    ayesha.lowerCollector();
                    ayesha.startCollector();
                })
                .build();

        ayesha.followTrajectoryAsync(enterWarehouseCycle5);

        while(opModeIsActive() && ayesha.isBusy()) {
            if(ayesha.retrieveCollectorDistance() < 3.3) {
                cycle5Collected = true;
                ayesha.breakFollowing();
                ayesha.setDrivePower(new Pose2d());

                exitWarehouseCycle5 = ayesha.trajectorySequenceBuilder(new Pose2d(ayesha.getPoseEstimate().getX(), 3, Math.toRadians(0)))
                        .setReversed(true)
                        //.splineTo(new Vector2d(25, 3), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-1.5, -21, Math.toRadians(70)), Math.toRadians(-110))
                        .build();

                completeCollection.start();
                ayesha.followTrajectorySequence(exitWarehouseCycle5);
                scoreFreightPreloaded.start();

                break;
            }
            ayesha.update();
        }

        if(!cycle5Collected) {
            jiggleCount++;

            ayesha.turn(Math.toRadians(-11));
            cycle5Collected = true;

            exitWarehouseCycle5 = ayesha.trajectorySequenceBuilder(new Pose2d(ayesha.getPoseEstimate().getX(), 3, Math.toRadians(-11)))
                    .setReversed(true)
                    .splineTo(new Vector2d(25, 3), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-1.5, -21, Math.toRadians(70)), Math.toRadians(-110))
                    .build();

            while(opModeIsActive() && exitWarehouseCycle5 == null) {}

            completeCollection.start();
            ayesha.followTrajectorySequence(exitWarehouseCycle5);
            scoreFreightPreloaded.start();
        }

        Trajectory enterWarehouseCycle6 = ayesha.trajectoryBuilder(exitWarehouseCycle5.end())
                .splineToSplineHeading(new Pose2d(32, 3, Math.toRadians(0)), Math.toRadians(0)) //36 for x
                .splineTo(new Vector2d(46, -6), Math.toRadians(-15))
                .addDisplacementMarker(0.18, 0, () -> {
                    ayesha.lowerDropper();
                    readyForCollectionNoStart.start();
                })
                .addDisplacementMarker(0.7, 0, () -> {
                    ayesha.lowerCollector();
                    ayesha.startCollector();
                })
                .build();

        ayesha.followTrajectoryAsync(enterWarehouseCycle6);

        while(opModeIsActive() && !isStopRequested()) {
            if(ayesha.retrieveCollectorDistance() < 3.3) {
                ayesha.breakFollowing();
                ayesha.setDrivePower(new Pose2d());

                exitWarehouseCycle6 = ayesha.trajectorySequenceBuilder(new Pose2d(ayesha.getPoseEstimate().getX(), ayesha.getPoseEstimate().getY(), ayesha.getPoseEstimate().getHeading()))
                        .setReversed(true)
                        .splineTo(new Vector2d(25, 3), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(0, -22.4, Math.toRadians(72.5)), Math.toRadians(-107.5))
                        .build();

                completeCollection.start();
                ayesha.followTrajectorySequence(exitWarehouseCycle6);
                scoreFreightPreloaded.start();

                break;
            }
            ayesha.update();
        }

        parkInWarehouse = ayesha.trajectoryBuilder(exitWarehouseCycle6.end())
                .splineToSplineHeading(new Pose2d(42, 3, Math.toRadians(0)), Math.toRadians(0),
                        AyeshaCancelable.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        AyeshaCancelable.getAccelerationConstraint(80))
                .build();

        ayesha.followTrajectory(parkInWarehouse);
    }

    private class LiftDropper extends Thread
    {
        private int level;

        public LiftDropper(int level) {
            this.level = level;
        }

        @Override
        public void run()
        {
            try
            {
                if(level == 3) {
                    ayesha.liftDropper();
                    ayesha.extendDropper(3);
                }
                else if(level == 2) {
                    ayesha.liftDropper();
                    ayesha.extendDropper(2);
                }
                else if(level == 1) {
                    ayesha.liftDropper();
                }
            }
            catch (Exception e) {}
        }
    }

    private class ScoreFreightCycle extends Thread
    {
        public ScoreFreightCycle() {

        }

        @Override
        public void run()
        {
            try
            {
                ayesha.liftBackDArm();
                sleep(40);
                ayesha.lowerDropper();
                sleep(100);
                ayesha.extendDropper(1);
                ayesha.liftFrontDArm();
            }
            catch (Exception e) {}
        }
    }

    private class ScoreFreightPreloaded extends Thread
    {
        public ScoreFreightPreloaded() {

        }

        @Override
        public void run()
        {
            try
            {
                ayesha.liftBackDArm();
                sleep(100);
                ayesha.lowerDropper();
                sleep(100);
                ayesha.extendDropper(1);
                ayesha.liftFrontDArm();
            }
            catch (Exception e) {}
        }
    }

    private class ReadyForCollection extends Thread
    {
        public ReadyForCollection() {}

        @Override
        public void run()
        {
            try
            {
                ayesha.readyForCollection();
            }
            catch (Exception e) {}
        }
    }

    private class ReadyForCollectionNoStart extends Thread
    {
        public ReadyForCollectionNoStart() {}

        @Override
        public void run()
        {
            try
            {
                ayesha.readyForCollectionNoStart();
            }
            catch (Exception e) {}
        }
    }


    private class CompleteCollection extends Thread
    {
        public CompleteCollection() { }

        @Override
        public void run()
        {
            try
            {
                ayesha.liftCollector();
                while(ayesha.retrieveCollectorRotationDistance() > 0.96) {

                }
                ayesha.reverseCollector();
                sleep(500);
                ayesha.lowerCollector();
                while(ayesha.retrieveCollectorRotationDistance() <= 0.96) {

                }
                sleep(100);
                ayesha.lowerFrontDArm();
                ayesha.stopCollector();
                sleep(100);
                ayesha.extendDropper(3);
                ayesha.liftDropper();
                ayesha.lowerCollector();

                this.interrupt();
            }
            catch (Exception e) {}
        }
    }

    private class RotateCapX extends Thread
    {
        public RotateCapX() { }

        @Override
        public void run()
        {
            try
            {
                ayesha.rotateCapX(-1);
                sleep(1425);
                ayesha.rotateCapX(0);

                this.interrupt();
            }
            catch (Exception e) {}
        }
    }

    private class SpinMeasuringTape extends Thread
    {
        public SpinMeasuringTape() { }

        @Override
        public void run()
        {
            try
            {
                ayesha.spinMeasuringTape(-1);
                sleep(2000);
                ayesha.spinMeasuringTape(0);

                this.interrupt();
            }
            catch (Exception e) {}
        }
    }
}