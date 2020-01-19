package org.igutech.autonomous.Vision;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Disabled
@Autonomous(group = "test")
public class PipeLineTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        OpenCvCamera phone_camera;

        SkyStonePipeLine stone_pipeline;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phone_camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phone_camera.openCameraDevice();

        stone_pipeline = new SkyStonePipeLine();
        phone_camera.setPipeline(stone_pipeline);
        phone_camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
        waitForStart();
        while (opModeIsActive()) {
            dashboardTelemetry.addData("LEFT RECT", stone_pipeline.left );
            dashboardTelemetry.addData("RIGHT RECT", stone_pipeline.right);
            dashboardTelemetry.addData("Pattern", stone_pipeline.pattern);
            dashboardTelemetry.update();
        }
        phone_camera.stopStreaming();

    }
}

