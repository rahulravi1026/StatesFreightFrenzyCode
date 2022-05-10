package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.AyeshaCancelable;
import org.firstinspires.ftc.teamcode.webcam.TSEPositionRedPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 Forward is positive x
 Backward is negative x
 Right is negative y
 Left is positive y
 */

@Disabled
@Autonomous(group = "drive")
public class Preloaded extends LinearOpMode {

    AyeshaCancelable ayesha;

    OpenCvCamera webcam;
    TSEPositionRedPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        ayesha = new AyeshaCancelable(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(-90));
        ayesha.setPoseEstimate(startPose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "backWebcam"), cameraMonitorViewId);

        pipeline = new TSEPositionRedPipeline();
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
        Thread completeCollection = new CompleteCollection();

        Thread scoreFreightCycle = new ScoreFreightCycle();
        Thread scoreFreightPreloaded = new ScoreFreightPreloaded();

        Trajectory placePreloadedShippingHubLevel3Or2 = ayesha.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-8.5, 16.5, Math.toRadians(-70)))
                .addDisplacementMarker(0.9, 0, () -> {
                    scoreFreightPreloaded.start();
                })
                .build();

        Trajectory placePreloadedShippingHubLevel1 = ayesha.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-8.5, 23, Math.toRadians(-70)))
                .addDisplacementMarker(0.9, 0, () -> {
                    scoreFreightPreloaded.start();
                })
                .build();

        TrajectorySequence enterWarehouseCycle1Level3Or2 = ayesha.trajectorySequenceBuilder(placePreloadedShippingHubLevel3Or2.end())
                .splineToSplineHeading(new Pose2d(33, -0.5, Math.toRadians(0)), Math.toRadians(-6))
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 23;
                    }
                })
                .splineTo(new Vector2d(55, -0.5), Math.toRadians(0))
                .addDisplacementMarker(0.18, 0, () -> {
                    readyForCollection.start();
                })
                .build();

        TrajectorySequence enterWarehouseCycle1Level1 = ayesha.trajectorySequenceBuilder(placePreloadedShippingHubLevel1.end())
                .splineToSplineHeading(new Pose2d(33, -0.5, Math.toRadians(0)), Math.toRadians(-6))
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 23;
                    }
                })
                .splineTo(new Vector2d(55, -0.5), Math.toRadians(0))
                .addDisplacementMarker(0.18, 0, () -> {
                    readyForCollection.start();
                })
                .build();

        waitForStart();

        if (isStopRequested()) return;

        if(pipeline.getAnalysis() == TSEPositionRedPipeline.TSEPosition.LEFT) { //Level 3
            liftDropperPreloaded3.start();
            ayesha.followTrajectory(placePreloadedShippingHubLevel3Or2);
            ayesha.followTrajectorySequence(enterWarehouseCycle1Level3Or2);
        }
        else if(pipeline.getAnalysis() == TSEPositionRedPipeline.TSEPosition.CENTER) { //Level 2
            liftDropperPreloaded2.start();
            ayesha.followTrajectory(placePreloadedShippingHubLevel3Or2);
            ayesha.followTrajectorySequence(enterWarehouseCycle1Level3Or2);
        }
        else if(pipeline.getAnalysis() == TSEPositionRedPipeline.TSEPosition.RIGHT) { //Level 1
            liftDropperPreloaded1.start();
            ayesha.followTrajectory(placePreloadedShippingHubLevel1);
            ayesha.followTrajectorySequence(enterWarehouseCycle1Level1);
        }
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
                    //ayesha.extendDropper(1);
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
                sleep(500);
                ayesha.lowerDropper();
                ayesha.extendDropper(1);
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
                sleep(450);
                ayesha.stopCollector();
                ayesha.lowerCollector();
                ayesha.lowerFrontDArm();
                ayesha.extendDropper(3);

                this.interrupt();
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
            }
            catch (Exception e) {}
        }
    }
}
