package org.firstinspires.ftc.teamcode.webcam;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TSEPositionRedPipeline extends OpenCvPipeline
{
    public enum TSEPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final Point CENTER_REGION__TOPLEFT_ANCHOR_POINT = new Point(1190,740);
    static final Point RIGHT_REGION_TOPLEFT_ANCHOR_POINT = new Point(665,740);
    static final int REGION_WIDTH = 80;
    static final int REGION_HEIGHT = 80;

    Point center_region_pointA = new Point(
            CENTER_REGION__TOPLEFT_ANCHOR_POINT.x,
            CENTER_REGION__TOPLEFT_ANCHOR_POINT.y);
    Point center_region_pointB = new Point(
            CENTER_REGION__TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            CENTER_REGION__TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Point right_region_pointA = new Point(
            RIGHT_REGION_TOPLEFT_ANCHOR_POINT.x,
            RIGHT_REGION_TOPLEFT_ANCHOR_POINT.y);
    Point right_region_pointB = new Point(
            RIGHT_REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            RIGHT_REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat center_region_Cb, right_region_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();

    private volatile TSEPosition position = TSEPosition.LEFT;

    private volatile int rightAvg;
    private volatile int centerAvg;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame)
    {
        inputToCb(firstFrame);

        center_region_Cb = Cb.submat(new Rect(center_region_pointA, center_region_pointB));
        right_region_Cb = Cb.submat(new Rect(right_region_pointA, right_region_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        inputToCb(input);

        centerAvg = (int) Core.mean(center_region_Cb).val[0];
        rightAvg = (int) Core.mean(right_region_Cb).val[0];

        Imgproc.rectangle(
                input,
                center_region_pointA,
                center_region_pointB,
                BLUE,
                2);

        Imgproc.rectangle(
                input,
                right_region_pointA,
                right_region_pointB,
                BLUE,
                2);

        if(centerAvg < 100)
        {
            position = TSEPosition.CENTER;

            Imgproc.rectangle(
                    input,
                    center_region_pointA,
                    center_region_pointB,
                    GREEN,
                    -1);
        }
        else if(rightAvg < 100)
        {
            position = TSEPosition.RIGHT;

            Imgproc.rectangle(
                    input,
                    right_region_pointA,
                    right_region_pointB,
                    GREEN,
                    -1);
        }
        else
        {
            position = TSEPosition.LEFT;
        }

        return input;
    }

    public TSEPosition getAnalysis()
    {
        return position;
    }

    public int getCenterAvg()
    {
        return centerAvg;
    }

    public int getRightAvg()
    {
        return rightAvg;
    }

}