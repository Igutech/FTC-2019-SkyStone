package org.igutech.autonomous.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.igutech.autonomous.roadrunner.Elevator;
import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.util.AutoCVUtil;
import org.igutech.autonomous.util.AutoUtilManager;
import org.opencv.core.Mat;

import kotlin.Unit;

@Config
@Autonomous(name = "HeadPIDTest", group = "igutech")
public class HeadPIDTest extends LinearOpMode {


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        Elevator elevator = new Elevator(hardwareMap);


        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "TestRoadrunner");
        manager.getDriveUtil().resetEncoders();
        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
        manager.getHardware().getServos().get("TransferServo").setPosition(0.43);
        manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);

        manager.getHardware().getServos().get("CapServo").setPosition(0.53);


        //intake facing bridge
        IguMecanumDriveBase drive = new IguMecanumDriveBase(manager);
        //drive.setPoseEstimate(new Pose2d(-40.0,60.0,Math.toRadians(0.0)));
        drive.setPoseEstimate(new Pose2d(-33.0, 63.0, Math.toRadians(0.0)));


        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        if (isStopRequested()) return;
        Trajectory patterAToFoundation = drive.trajectoryBuilder()
                .lineTo(new Vector2d(10,63), new ConstantInterpolator(0))
                .build();
        drive.followTrajectorySync(patterAToFoundation);
        drive.waitForSync(100000);


        while (!isStopRequested() && drive.isBusy()) {
            Pose2d currentPose = drive.getPoseEstimate();
            dashboardTelemetry.addData("x", currentPose.getX());
            dashboardTelemetry.addData("y", currentPose.getY());
            dashboardTelemetry.addData("heading", currentPose.getHeading());
            dashboardTelemetry.addData("x error", drive.getLastError().getX());
            dashboardTelemetry.addData("y error", drive.getLastError().getY());
            dashboardTelemetry.addData("heading error", drive.getLastError().getHeading());
            dashboardTelemetry.update();
            drive.update();
        }

    }

}
