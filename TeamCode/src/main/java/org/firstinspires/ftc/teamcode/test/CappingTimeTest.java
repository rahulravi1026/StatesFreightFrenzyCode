package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Ayesha;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */

@Disabled
@TeleOp(group = "drive")
public class CappingTimeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Ayesha ayesha = new Ayesha(hardwareMap);

        ayesha.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        ayesha.rotateCapX(-1);
        sleep(1275);
        ayesha.rotateCapX(0);

        ayesha.spinMeasuringTape(1);
        sleep(2000);
        ayesha.spinMeasuringTape(0);
    }
}