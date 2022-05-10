package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AyeshaCancelable;

/**
 * Example opmode demonstrating how to break out from a live trajectory at any arbitrary point in
 * time. This will allow you to do some cool things like incorporating live trajectory following in
 * your teleop. Check out TeleOpAgumentedDriving.java for an example of such behavior.
 * <p>
 * 3 seconds into the start of the opmode, `drive.breakFollowing()` is called, breaking out of all
 * trajectory following.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java and TrajectorySequenceRunnerCancelable.java
 * classes. Please ensure that these files are copied into your own project.
 */

@Disabled
@Autonomous(group = "advanced")
public class AutoBreakTrajectory extends LinearOpMode {

    AyeshaCancelable ayesha;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        ayesha = new AyeshaCancelable(hardwareMap);

        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field
        // This example sets the bot at x: -20, y: -35, and facing 90 degrees (turned counter-clockwise)
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        ayesha.setPoseEstimate(startPose);

        Thread completeCollection = new CompleteCollection();

        waitForStart();

        if (isStopRequested()) return;

        // Example spline path from SplineTest.java
        Trajectory traj1 = ayesha.trajectoryBuilder(startPose)
                .forward(50)
//                        AyeshaCancelable.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        AyeshaCancelable.getAccelerationConstraint(20))
                .build();

//        Trajectory traj2 = ayesha.trajectoryBuilder(ayesha.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
////                        AyeshaCancelable.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
////                        AyeshaCancelable.getAccelerationConstraint(20))
//                .build();

        // We follow the trajectory asynchronously so we can run our own logic
        //ayesha.lowerCollector();
        ayesha.startCollector();

        ayesha.followTrajectoryAsync(traj1);

        // Start the timer so we know when to cancel the following
        //ElapsedTime stopTimer = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()) {
            // 3 seconds into the opmode, we cancel the following
            if (ayesha.retrieveCollectorDistance() < 3) {
                // Cancel following
                ayesha.breakFollowing();
                ayesha.setDrivePower(new Pose2d());
                ayesha.stopCollector();

                //ayesha.update();

//                Pose2d poseEstimate = ayesha.getPoseEstimate();
//                telemetry.addData("x", poseEstimate.getX());
//                telemetry.addData("y", poseEstimate.getY());
//                telemetry.addData("heading", poseEstimate.getHeading());
//                telemetry.update();
//
//                Trajectory traj2 = ayesha.trajectoryBuilder(ayesha.getPoseEstimate())
//                        .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)),
//                        AyeshaCancelable.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        AyeshaCancelable.getAccelerationConstraint(30))
//                        .build();
//
//                sleep(250);
//
//                ayesha.followTrajectory(traj2);
//
//                sleep(1000);


                // Stop the motors
                //ayesha.setDrivePower(new Pose2d());
                //ayesha.stopCollector();
                //ayesha.update();
                //completeCollection.start();
                break;
            }
            ayesha.update();
        }

//        Trajectory traj2 = ayesha.trajectoryBuilder(ayesha.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
////                        AyeshaCancelable.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
////                        AyeshaCancelable.getAccelerationConstraint(20))
//                .build();
//
//        ayesha.followTrajectory(traj2);
//
//        sleep(1000);

        // Update drive
        //ayesha.update();


//
//        Pose2d poseEstimate = ayesha.getPoseEstimate();
//        telemetry.addData("x", poseEstimate.getX());
//        telemetry.addData("y", poseEstimate.getY());
//        telemetry.addData("heading", poseEstimate.getHeading());
//        telemetry.update();

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
                sleep(300);
                ayesha.stopCollector();
                ayesha.lowerCollector();
                ayesha.lowerFrontDArm();
                ayesha.extendDropper(3);
                ayesha.liftDropper();

                this.interrupt();
            }
            catch (Exception e) {}
        }
    }
}
