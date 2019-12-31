package org.igutech.autonomous;

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
@Autonomous(name = "RoadRunnerTestOp", group = "igutech")
public class RoadRunnerTestOp extends LinearOpMode {

    public static int strafeRight = 15;
    public static int x = -41;
    public static int y = 30;
    public static int forward = 80;


    public static int angle = 0;
    public static int angle2 = 70;


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
                .addMarker(() -> {
                    manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                    manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                    return Unit.INSTANCE;
                })
                .back(10.0)
                .strafeRight(15.0)
                .lineTo(new Vector2d(-43, 25.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(-130)))
                .lineTo(new Vector2d(-43, 10.0), new LinearInterpolator(Math.toRadians(-130), Math.toRadians(0.0)))
//                .addMarker(7.0, () -> {
//                    manager.getHardware().getMotors().get("left_intake").setPower(0.0);
//                    manager.getHardware().getMotors().get("right_intake").setPower(-0.0);
//                    return Unit.INSTANCE;
//                })
//                .addMarker(7.5, () -> {
//                    manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
//                    return Unit.INSTANCE;
//                })
//                .addMarker(8.5, () -> {
//                    manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
//                    return Unit.INSTANCE;
//                })
                .lineTo(new Vector2d(-43.0, 40.0), new LinearInterpolator(Math.toRadians(-130.0), Math.toRadians(0.0)))
                .lineTo(new Vector2d(-42.0, 35), new LinearInterpolator(Math.toRadians(-130.0), Math.toRadians(130)))
                .lineTo(new Vector2d(15.0, 30), new LinearInterpolator(Math.toRadians(-30), Math.toRadians(angle)))
                //.lineTo(new Vector2d(45.0, 38.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(angle2)))
                //.lineTo(new Vector2d(45.0, 30.0), new LinearInterpolator(Math.toRadians(angle2), Math.toRadians(0.0)))
                .build();
        drive.followTrajectorySync(patterAToFoundation);

        //move foundation
//        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
//        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
//        drive.waitForSync(1.0);
//        Trajectory patternAMoveFoundation = drive.trajectoryBuilder()
//                .forward(35)
//                .build();
//        drive.followTrajectorySync(patternAMoveFoundation);
//        drive.turnSync(Math.toRadians(350));
//        //finish moving foundation
//
//        //place stone
//        manager.getHardware().getMotors().get("stoneElevator").setPower(0.6);
//        sleep(600);
//        manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
//        sleep(400);
//        manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
//        sleep(400);
//        manager.getHardware().getMotors().get("stoneElevator").setPower(-0.6);
//        sleep(600);
//        manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
//        manager.getHardware().getMotors().get("stoneElevator").setPower(0.6);
//        sleep(600);
//        manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
//        sleep(400);
//        manager.getHardware().getServos().get("RotationServo").setPosition(0.28);
//        sleep(600);
//        manager.getHardware().getMotors().get("stoneElevator").setPower(-0.6);
//        sleep(600);
//        manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
//        //finish placing stone
//
//        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
//        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
//
//        drive.setPoseEstimate(new Pose2d(40.0, 38.0, Math.toRadians(-180.0)));
//        Trajectory three = drive.trajectoryBuilder()
//                .addMarker(() -> {
//                    manager.getHardware().getMotors().get("left_intake").setPower(0.75);
//                    manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
//                    return Unit.INSTANCE;
//                })
//                .forward(45.0)
//                .lineTo(new Vector2d(-40.0,35.0), new LinearInterpolator(Math.toRadians(-180.0), Math.toRadians(180.0)))
//                .lineTo(new Vector2d(-40.0,20.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(0.0)))
//                .lineTo(new Vector2d(-30.0,20.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(0.0)))
//                //finish intaking second skystone
//                .lineTo(new Vector2d(-30.0,37.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(0.0)))
//                .lineTo(new Vector2d(20.0, 37.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(0.0)))
//                .lineTo(new Vector2d(40.0, 37.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(180.0)))
//                .lineTo(new Vector2d(50.0, 37.0), new LinearInterpolator(Math.toRadians(180.0), Math.toRadians(0.0)))
//                .build();
//        drive.followTrajectorySync(three);

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
