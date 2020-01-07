package org.igutech.autonomous.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.igutech.autonomous.LABPipeLine;
import org.igutech.config.Hardware;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


public class AutoCVUtil {
    private Hardware hardware;
    OpenCvCamera phone_camera;

    LABPipeLine stone_pipeline;
    ;
    public AutoCVUtil(AutoUtilManager manager, Hardware hardware) {
        this.hardware = hardware;

        int cameraMonitorViewId = hardware.getHardwareMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardware.getHardwareMap().appContext.getPackageName());
        phone_camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phone_camera.openCameraDevice();
        stone_pipeline = new LABPipeLine();
        phone_camera.setPipeline(stone_pipeline);


    }
    public void activate(){phone_camera.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);}
    public void shutdown(){ phone_camera.stopStreaming();}
    public Pattern getPattern()
    {
        if(stone_pipeline.pattern==1)
            return Pattern.PATTERN_A;
        if(stone_pipeline.pattern==2)
            return Pattern.PATTERN_B;
        if(stone_pipeline.pattern==3)
            return Pattern.PATTERN_C;
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

