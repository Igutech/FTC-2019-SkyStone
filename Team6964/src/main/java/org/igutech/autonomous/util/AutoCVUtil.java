package org.igutech.autonomous.util;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.igutech.config.Hardware;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.opencv.core.CvType.CV_8UC1;

public class AutoCVUtil {
    private Hardware hardware;
     OpenCvCamera phone_camera;

     SamplePipeline stone_pipeline;
//     FtcDashboard dashboard = FtcDashboard.getInstance();
//     Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public AutoCVUtil(AutoUtilManager manager, Hardware hardware) {
        this.hardware = hardware;

        int cameraMonitorViewId = hardware.getHardwareMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardware.getHardwareMap().appContext.getPackageName());
        phone_camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //phone_camera = new OpenCvWebcam(hardware.getHardwareMap().get(WebcamName.class,"Webcam"),cameraMonitorViewId);

        phone_camera.openCameraDevice();
        stone_pipeline = new SamplePipeline();
        phone_camera.setPipeline(stone_pipeline);


    }
    public void activate(){phone_camera.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);}
    public void shutdown(){ phone_camera.stopStreaming();}
    public Pattern getPattern()
    {
        if(stone_pipeline.pattern==1)
            return Pattern.PATTERN_C;
        if(stone_pipeline.pattern==2)
            return Pattern.PATTERN_B;
        if(stone_pipeline.pattern==3)
            return Pattern.PATTERN_A;
        else
            return Pattern.UNKNOWN;

    }


    public enum Pattern {
        PATTERN_A,
        PATTERN_B,
        PATTERN_C,
        UNKNOWN
    }
}

class SamplePipeline extends OpenCvPipeline {
    int left_hue;
    int right_hue;

    int left_br;
    int right_br;

    int pattern;
    //    public static double left_one=9f / 32f;
//    public static double left_two=(5f / 32f);
//    public static double left_three=(15f / 32f);
//    public static double left_four=(7f / 32f);
//

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
//    public static double right_one=0.5;
//    public static double right_two=0.156;
//    public static double right_three=0.468;
//    public static double right_four=0.218;


    @Override
    public Mat processFrame(Mat input) {

        input.convertTo(input, CV_8UC1, 1, 10);

//        int[] left_rect = {
//                (int) (input.cols() * left_one),
//                (int) (input.rows() * left_two),
//                (int) (input.cols() * left_three ),
//                (int) (input.rows() * left_four)
//        };
        int[] left_rect = {
                left_one,
                left_two,
                left_three,
                left_four
        };
        dashboardTelemetry.addData("1", (int) (input.cols() * left_one));
        dashboardTelemetry.addData("2", (int) (input.rows() * left_two));
        dashboardTelemetry.addData("3", (int) (input.cols() * left_three));
        dashboardTelemetry.addData("4", (int) (input.rows() * left_four));
        dashboardTelemetry.update();


//        int[] right_rect = {
//                (int) (input.cols() * right_one),
//                (int) (input.rows() * right_two),
//                (int) (input.cols() * right_three),
//                (int) (input.rows() * right_four)
//        };

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

        Mat left_block = input.submat(left_rect[1], left_rect[3], left_rect[0], left_rect[2]);
        Mat right_block = input.submat(right_rect[1], right_rect[3], right_rect[0], right_rect[2]);


        Scalar left_mean = Core.mean(left_block);


        Scalar right_mean = Core.mean(right_block);

        left_hue = get_hue((int) left_mean.val[0], (int) left_mean.val[1], (int) left_mean.val[2]);
        right_hue = get_hue((int) right_mean.val[0], (int) right_mean.val[1], (int) right_mean.val[2]);
        left_br = get_brightness((int) left_mean.val[0], (int) left_mean.val[1], (int) left_mean.val[2]);
        right_br = get_brightness((int) right_mean.val[0], (int) right_mean.val[1], (int) right_mean.val[2]);

        //Patern 1 = Stone Stone Skystone
        //Pattern 2 = Stone Skystone Stone
        //Pattern 3 = Skystone Stone Stone
        if (left_br > 100 && right_br > 100) pattern = 3;
        else if (left_br > 100 && right_br < 100) pattern = 1;
        else if (left_br < 100 && right_br > 100) pattern = 2;
        else if (left_br < 100 && right_br < 100) {
            if (left_br > right_br) {
                pattern = 2;
            } else if (left_br < right_br) {
                pattern = 1;
            } else {
                pattern = 3;
            }
        }

        return input;
    }


    private int get_hue(int red, int green, int blue) {

        float min = Math.min(Math.min(red, green), blue);
        float max = Math.max(Math.max(red, green), blue);

        if (min == max) {
            return 0;
        }

        float hue = 0f;
        if (max == red) {
            hue = (green - blue) / (max - min);

        } else if (max == green) {
            hue = 2f + (blue - red) / (max - min);

        } else {
            hue = 4f + (red - green) / (max - min);
        }

        hue = hue * 60;
        if (hue < 0) hue = hue + 360;

        return Math.round(hue);
    }

    private int get_brightness(int red, int green, int blue) {
        return (int) (((double) (red + green + blue)) / 3);
    }
}
