package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Ayesha;

@TeleOp(group = "drive")
public class RedTeleOp extends LinearOpMode {

    Ayesha ayesha;

    String level = "three";

    String mode = "TeleOp";

    @Override
    public void runOpMode() throws InterruptedException {
        ayesha = new Ayesha(hardwareMap);
        ayesha.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Thread completeCollection = new CompleteCollection();
        Thread completeDeposition = new CompleteDeposition();
        Thread completeDepositionDuck = new CompleteDepositionDuck();
        Thread carousel = new Carousel();

        waitForStart();

        ayesha.readyForCollection();

        while (!isStopRequested()) {

            /**
             * Chassis Controls
             */

            ayesha.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x * 0.9
                    )
            );
            ayesha.update();


            /**
             * Collector Controls
             */

            if(ayesha.freightCollected() && !completeCollection.isAlive()) {
                gamepad1.rumble(500);
                completeCollection.start();
            }

            if(gamepad1.left_bumper) {
                ayesha.readyForCollection();
            }
            if(gamepad1.right_bumper) {
                ayesha.liftCollector();
            }

            if(gamepad1.square) {
                ayesha.stopCollector();
            }

            if(gamepad1.triangle) {
                ayesha.duckCollector();
                mode = "End Game";
            }

            if(gamepad1.dpad_up && !completeCollection.isAlive()) {
                completeCollection.start();
            }

            /**
             * Dropper Controls
             */

            if(gamepad1.right_trigger >= 0.2 && !completeDeposition.isAlive() && mode.equals("TeleOp")) {
                ayesha.liftBackDArm();
                completeDeposition.start();
            }

            if(gamepad1.right_trigger >= 0.2 && !completeDepositionDuck.isAlive() && mode.equals("End Game")) {
                ayesha.liftBackDArm();
                completeDepositionDuck.start();
            }

            if(gamepad1.cross) {
                ayesha.lowerDropper();
                ayesha.extendDropper(1);
                ayesha.readyForCollection();
            }

            if(gamepad1.touchpad_finger_1) {
                if(gamepad1.touchpad_finger_1_x <= -0.5) {
                    level = "two";
                }
                else if(gamepad1.touchpad_finger_1_x >= 0.5) {
                    level = "three";
                }
             }

            /**
             * Carousel Controls
             */

            if(gamepad1.circle)
                carousel.start();

            /**
             * Capping Controls
             */

            float leftStickY = gamepad2.left_stick_y;
            float rightStickY = gamepad2.right_stick_y;
            float rightStickX = -gamepad2.right_stick_x;

            leftStickY = (float) scaleInput(leftStickY);
            rightStickY = (float) scaleInput(rightStickY);
            rightStickX = (float) scaleInput(rightStickX);

            ayesha.spinMeasuringTape(leftStickY);
            ayesha.rotateCapY(rightStickY * 0.2);
            ayesha.rotateCapX(rightStickX * 0.2);

            telemetry.addData("level", level);
            telemetry.addData("mode", mode);


            telemetry.update();

        }

    }

    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale ;
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
                    if(level.equals("three")) {
                        ayesha.extendDropper(3);
                    }
                    if(level.equals("two")) {
                        ayesha.extendDropper(2);
                    }
                    ayesha.liftDropper();
                    ayesha.lowerCollector();

                    this.interrupt();
            }
            catch (Exception e) {}
        }
    }

    private class CompleteDeposition extends Thread
    {
        public CompleteDeposition() { }

        @Override
        public void run()
        {
            try
            {
                sleep(300);
                ayesha.lowerDropper();
                ayesha.extendDropper(1);
                ayesha.readyForCollectionNoFrontArm();
                sleep(500);
                ayesha.liftFrontDArm();
                this.interrupt();
            }
            catch (Exception e) {}
        }
    }

    private class CompleteDepositionDuck extends Thread
    {
        public CompleteDepositionDuck() { }

        @Override
        public void run()
        {
            try
            {
                sleep(300);
                ayesha.lowerDropper();
                ayesha.extendDropper(1);
                ayesha.readyForCollectionDuck();
                sleep(500);
                ayesha.liftFrontDArm();
                this.interrupt();
            }
            catch (Exception e) {}
        }
    }

    private class Carousel extends Thread
    {
        public Carousel() { }

        @Override
        public void run()
        {
            try
            {
                ayesha.deliver9DucksRed();

                this.interrupt();
            }
            catch (Exception e) {}
        }
    }
}
