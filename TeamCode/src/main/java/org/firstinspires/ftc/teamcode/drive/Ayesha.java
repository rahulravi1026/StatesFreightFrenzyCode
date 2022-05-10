package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import static java.lang.Thread.sleep;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class Ayesha extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private DcMotorEx dropperFront, dropperBack, carousel;
    private DcMotor collector;

    private Servo leftCR, rightCR;
    private Servo leftDR, rightDR, backDArm, frontDArm;
    private CRServo leftMeasuringTape, rightMeasuringTape, rotateCapX, rotateCapY;

    private DistanceSensor collectorDistanceSensor, collectorRotationSensor;
    private ColorSensor collectorColorSensor;

    private List<DcMotorEx> motors;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public static double leftCRDown = 0.69;
    public static double leftCRUp = 0.04;

    public static double rightCRDown = 0.31;
    public static double rightCRUp = 0.96;

    public static double leftDRDown = 0.47;
    public static double leftDRUp = 0.105;

    public static double rightDRDown = 0.53;
    public static double rightDRUp = 0.895;

    public static double backDArmDown = 0;
    public static double backDArmUp = 0.085;

    public static double frontDArmDown = 0.4;
    public static double frontDArmUp = 0.065;

    public static double level3 = 660;
    public static double level2 = 250;
    public static double level1 = 20;

    public static double oneCarouselSpin = 805;

    public static double carouselRedPowerInitial = 0.31;
    public static double carouselRedPowerMiddle = 0.43;
    public static double carouselRedPowerFinal = 0.52;

    public static double carouselBluePowerInitial = -0.31;
    public static double carouselBluePowerMiddle = -0.43;
    public static double carouselBluePowerFinal = -0.52;

    public static double firstEncoderCount = 500;
    public static double middleEncoderCount = 160;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;

    public Ayesha(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        collector = hardwareMap.get(DcMotor.class, "collector");
        dropperFront = hardwareMap.get(DcMotorEx.class, "dropperFront");
        dropperBack = hardwareMap.get(DcMotorEx.class, "dropperBack");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

        leftCR = hardwareMap.get(Servo.class, "leftCR");
        rightCR = hardwareMap.get(Servo.class, "rightCR");
        leftDR = hardwareMap.get(Servo.class, "leftDR");
        rightDR = hardwareMap.get(Servo.class, "rightDR");
        backDArm = hardwareMap.get(Servo.class, "backDArm");
        frontDArm = hardwareMap.get(Servo.class, "frontDArm");

        leftMeasuringTape = hardwareMap.get(CRServo.class, "leftMeasuringTape");
        rightMeasuringTape = hardwareMap.get(CRServo.class, "rightMeasuringTape");
        rotateCapX = hardwareMap.get(CRServo.class, "rotateCapX");
        rotateCapY = hardwareMap.get(CRServo.class, "rotateCapY");

        collectorDistanceSensor = hardwareMap.get(DistanceSensor.class, "collectorSensor");
        collectorColorSensor = hardwareMap.get(ColorSensor.class, "collectorSensor");

        collectorRotationSensor = hardwareMap.get(DistanceSensor.class, "collectorRotationSensor");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        collector.setDirection(DcMotorSimple.Direction.REVERSE);
//        carousel.setDirection(DcMotorSimple.Direction.REVERSE);

//        leftMeasuringTape.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMeasuringTape.setDirection(DcMotorSimple.Direction.REVERSE);
//        rotateCapX.setDirection(DcMotorSimple.Direction.REVERSE);
//        rotateCapY.setDirection(DcMotorSimple.Direction.REVERSE);

        leftCR.setPosition(leftCRDown);
        rightCR.setPosition(rightCRDown);

        leftDR.setPosition(leftDRDown);
        rightDR.setPosition(rightDRDown);

        backDArm.setPosition(backDArmDown);
        frontDArm.setPosition(frontDArmUp);

        dropperFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dropperFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dropperFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dropperBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dropperBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dropperBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMeasuringTape.setPower(0);
        rightMeasuringTape.setPower(0);
        rotateCapX.setPower(0);
        rotateCapY.setPower(0);

        // TODO: if desired, use setLocalizer() to change the localization method
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    /**
     Servo Motion Functions
     */

    public void liftCollector() {
        leftCR.setPosition(leftCRUp);
        rightCR.setPosition(rightCRUp);
    }

    public void lowerCollector() {
        leftCR.setPosition(leftCRDown);
        rightCR.setPosition(rightCRDown);
    }

    public void liftDropper() {
        leftDR.setPosition(leftDRUp);
        rightDR.setPosition(rightDRUp);
    }

    public void lowerDropper() {
        leftDR.setPosition(leftDRDown);
        rightDR.setPosition(rightDRDown);
    }

    public void liftBackDArm() {
        backDArm.setPosition(backDArmUp);
    }

    public void lowerBackDArm() {
        backDArm.setPosition(backDArmDown);
    }

    public void liftFrontDArm() {
        frontDArm.setPosition(frontDArmUp);
    }

    public void lowerFrontDArm() {
        frontDArm.setPosition(frontDArmDown);
    }

    /**
     Collector Functions
     */

    public void startCollector() {
        collector.setPower(1);
    }

    public void stopCollector() {
        collector.setPower(0);
    }

    public void reverseCollector() {
        collector.setPower(-1);
    }

    public void duckCollector() { collector.setPower(0.5); }

    public void readyForCollection() {
        startCollector();
        lowerCollector();
        lowerDropper();
        liftFrontDArm();
        lowerBackDArm();
    }

    public void readyForCollectionNoFrontArm() {
        startCollector();
        lowerCollector();
        lowerDropper();
        lowerBackDArm();
    }

    public void readyForCollectionDuck() {
        duckCollector();
        lowerCollector();
        lowerDropper();
        lowerBackDArm();
    }

    public boolean freightCollected() {
        if(retrieveCollectorDistance() < 3) {
            return true;
        }
        return false;
    }

    /**
     Dropper Functions
     */

    public void extendDropper(int level) {
        if (level == 3) {
            dropperFront.setPower(0.85);
            dropperBack.setPower(0.85);
            while(dropperFront.getCurrentPosition() < level3) {
                if(dropperFront.getCurrentPosition() >= (level3 * 0.05)) {
                    liftDropper();
                }
            }
            dropperFront.setPower(0.1);
            dropperBack.setPower(0.1);
        }
        else if (level == 2) {
            dropperFront.setPower(1);
            dropperBack.setPower(1);
            while(dropperFront.getCurrentPosition() < level2) {
                if (dropperFront.getCurrentPosition() >= (level2 * 0.1)) {
                    liftDropper();
                }
            }
            dropperFront.setPower(0.1);
            dropperBack.setPower(0.1);
        }
        else if (level == 1) {
            dropperFront.setPower(-0.5);
            dropperBack.setPower(-0.5);
            while (dropperFront.getCurrentPosition() > level1) {}
            dropperFront.setPower(0);
            dropperBack.setPower(0);
        }
    }

    /**
     Carousel Functions
     */

    public void spinCarouselBlue(double encoder) {
        carousel.setPower(carouselBluePowerInitial);
        while(Math.abs(carousel.getCurrentPosition()) < (encoder - middleEncoderCount)) {}
        carousel.setPower(carouselBluePowerFinal);
        while(Math.abs(carousel.getCurrentPosition()) < encoder) {}
        carousel.setPower(0);
    }

    public void deliver9DucksBlue() throws InterruptedException {
        stopCollector();
        int i = 1;
        while(i < 10) {
            spinCarouselBlue(oneCarouselSpin * i);
            sleep(825);
            i++;
        }
        carousel.setPower(0);
    }

    public void spinCarouselRed(double encoder) {
        carousel.setPower(carouselRedPowerInitial);
        while(Math.abs(carousel.getCurrentPosition()) < (encoder - middleEncoderCount)) {}
        carousel.setPower(carouselRedPowerFinal);
        while(Math.abs(carousel.getCurrentPosition()) < encoder) {}
        carousel.setPower(0);
    }

    public void deliver9DucksRed() throws InterruptedException {
        stopCollector();
        int i = 1;
        while(i < 10) {
            spinCarouselRed(oneCarouselSpin * i);
            sleep(825);
            i++;
        }
        carousel.setPower(0);
    }

    /**
     Capping Functions
     */

    public void spinMeasuringTape(double power) {
        leftMeasuringTape.setPower(power);
        rightMeasuringTape.setPower(power);
    }

    public void rotateCapY(double power) {
        rotateCapY.setPower(power);
    }

    public void rotateCapX(double power) {
        rotateCapX.setPower(power);
    }

    /**
     Retrieving Values Functions
     */

    public double retrieveDropperFrontEncoder() {
        return dropperFront.getCurrentPosition();
    }

    public double retrieveDropperBackEncoder() {
        return dropperBack.getCurrentPosition();
    }

    public double retrieveCarouselEncoder() {
        return carousel.getCurrentPosition();
    }

    public double retrieveColor() {
        Color.RGBToHSV((int) (collectorColorSensor.red() * SCALE_FACTOR),
                (int) (collectorColorSensor.green() * SCALE_FACTOR),
                (int) (collectorColorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        return hsvValues[0];
    }

    public double retrieveCollectorDistance() { return collectorDistanceSensor.getDistance(DistanceUnit.CM); }

    public double retrieveCollectorRotationDistance() { return collectorRotationSensor.getDistance(DistanceUnit.CM); }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // To work around an SDK bug, use -zRotationRate in place of xRotationRate
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as 
        // expected). This bug does NOT affect orientation. 
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return (double) -imu.getAngularVelocity().xRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
