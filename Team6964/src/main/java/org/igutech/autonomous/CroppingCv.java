package org.igutech.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.igutech.autonomous.SamplePipeLine;

@Autonomous(group = "test")
public class CroppingCv extends LinearOpMode {
    @Override
    public void runOpMode() {
        OpenCvCamera phone_camera;

        SamplePipeLine stone_pipeline;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phone_camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phone_camera.openCameraDevice();

        stone_pipeline = new SamplePipeLine();
        phone_camera.setPipeline(stone_pipeline);
        phone_camera.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("FRAME", phone_camera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phone_camera.getFps()));
            telemetry.addData("TFT MS", phone_camera.getTotalFrameTimeMs());
            telemetry.addData("PT MS", phone_camera.getPipelineTimeMs());
            telemetry.addData("OT MS", phone_camera.getOverheadTimeMs());
            telemetry.addData("MAX FPS", phone_camera.getCurrentPipelineMaxFps());
            telemetry.addData("LEFT RECT", stone_pipeline.left_hue + " " + stone_pipeline.left_br);
            telemetry.addData("RIGHT RECT", stone_pipeline.right_hue + " " + stone_pipeline.right_br);
            telemetry.addData("PATTERN", stone_pipeline.pattern);
            telemetry.update();
            dashboardTelemetry.addData("FRAME", phone_camera.getFrameCount());
            dashboardTelemetry.addData("FPS", String.format("%.2f", phone_camera.getFps()));
            dashboardTelemetry.addData("TFT MS", phone_camera.getTotalFrameTimeMs());
            dashboardTelemetry.addData("PT MS", phone_camera.getPipelineTimeMs());
            dashboardTelemetry.addData("OT MS", phone_camera.getOverheadTimeMs());
            dashboardTelemetry.addData("MAX FPS", phone_camera.getCurrentPipelineMaxFps());
            dashboardTelemetry.addData("LEFT RECT", stone_pipeline.left_hue + " " + stone_pipeline.left_br);
            dashboardTelemetry.addData("RIGHT RECT", stone_pipeline.right_hue + " " + stone_pipeline.right_br);
            dashboardTelemetry.addData("PATTERN", stone_pipeline.pattern);
            dashboardTelemetry.update();
        }
        phone_camera.stopStreaming();

    }
}
