package org.igutech.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class LABPipeLine extends OpenCvPipeline {
    private Mat matYCrCb = new Mat();
    private Mat matCb_left = new Mat();
    private Mat matCb_Right = new Mat();
    private Mat left_block = new Mat();
    private Mat right_block = new Mat();

    public double left;
    public double right;

    public int pattern=2;


    public static int right_one = 160;
    public static int right_two = 190;
    public static int right_three = 210;
    public static int right_four = 210;

    public static int left_one = 90;
    public static int left_two = 190;
    public static int left_three = 130;
    public static int left_four = 210;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);
        int[] left_rect = {
                left_one,
                left_two,
                left_three ,
                left_four
        };

        int[] right_rect = {
                right_one,
                right_two,
                right_three,
                right_four
        };
        Imgproc.rectangle(
                input,
                new Point(
                        left_rect[0],
                        left_rect[1]),

                new Point(
                        left_rect[2],
                        left_rect[3]),
                new Scalar(0, 255, 0), 1);

        Imgproc.rectangle(
                input,
                new Point(
                        right_rect[0],
                        right_rect[1]),

                new Point(
                        right_rect[2],
                        right_rect[3]),
                new Scalar(0, 0, 255), 1);

       //TODO check if checking if it is null makes it draw on the same frame or the updated one
            left_block = matYCrCb.submat(left_rect[1], left_rect[3], left_rect[0], left_rect[2]);
            right_block = matYCrCb.submat(right_rect[1], right_rect[3], right_rect[0], right_rect[2]);


        Core.extractChannel(left_block, matCb_left, 2);
        Core.extractChannel(right_block, matCb_Right, 2);

        Scalar left_mean = Core.mean(matCb_left);
        Scalar right_mean = Core.mean(matCb_Right);
        left=left_mean.val[0];
        right=right_mean.val[0];

        if(left-right>=45){pattern=1;}
        if(right-left>=45){pattern=2;}
        if(Math.abs(left-right)<=15){pattern=3;}

        return input;

    }
}