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
import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.util.AutoCVUtil;
import org.igutech.autonomous.util.AutoUtilManager;
import org.opencv.core.Mat;

import kotlin.Unit;
@Config
@Autonomous(name="RoadRunnerTestOp", group="igutech")
public class RoadRunnerTestOp extends LinearOpMode {

    public static int angle = -140 ;
    public static int strafeRight  = 15;
    public static int x = -50;
    public static int y = 25;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {

        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "TestRoadrunner");
        manager.getDriveUtil().resetEncoders();


        //intake facing bridge
        IguMecanumDriveBase drive = new IguMecanumDriveBase(manager);
        drive.setPoseEstimate(new Pose2d(-40.0,60.0,Math.toRadians(0.0)));


        Trajectory test = drive.trajectoryBuilder()
                .addMarker(() -> {
                    manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                    manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                    return Unit.INSTANCE;
                })
                .back(10.0)
                .strafeRight(strafeRight)
                .lineTo(new Vector2d(x,y), new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(angle)))
                .lineTo(new Vector2d(x,15), new LinearInterpolator(Math.toRadians(angle),Math.toRadians(0)))
                .lineTo( new Vector2d(-40.0,40.0), new LinearInterpolator(Math.toRadians(angle),Math.toRadians(0.0)))

                .addMarker(7.0,() -> {
                    manager.getHardware().getMotors().get("left_intake").setPower(0.0);
                    manager.getHardware().getMotors().get("right_intake").setPower(-0.0);
                    return Unit.INSTANCE;
                })
                .addMarker(7.5,() -> {
                    manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                    return Unit.INSTANCE;
                })
                .addMarker(8.5,() -> {
                    manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(45.0,40.0), new LinearInterpolator(Math.toRadians(angle),Math.toRadians(-125.0)))
                .lineTo( new Vector2d(45.0,30.0), new LinearInterpolator(Math.toRadians(100.0),Math.toRadians(-10.0)))

                // .lineTo(new Vector2d(50.0,40.0), new LinearInterpolator(Math.toRadians(angle),Math.toRadians(-125.0)))
               // .lineTo( new Vector2d(50.0,25.0), new LinearInterpolator(Math.toRadians(100.0),Math.toRadians(-10.0)))



//                .addMarker(() -> {
//                    manager.getHardware().getMotors().get("left_intake").setPower(0.75);
//                    manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
//                    return Unit.INSTANCE;
//                })
//                .strafeRight(25.0)
//
//                .lineTo(new Vector2d(-15, 30), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(-160)))
//                .lineTo(new Vector2d(-28, 16), new LinearInterpolator(Math.toRadians(-160),Math.toRadians(0.0)))
//                .lineTo(new Vector2d(-25.0, 15.0),  new LinearInterpolator(Math.toRadians(-160.0),Math.toRadians(70)))
//                .strafeRight(20)
//                .back(30.0)
               // .lineTo( new Vector2d(-28.0, 10.0))


//                .strafeRight(40)
//                .strafeRight(10.0)
//                .back(20.0)
            //  .lineTo(new Vector2d(x, y), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(angle)))


                //.forward(5.0)
               // .lineTo(new Vector2d(-25.0, 20.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(-10)))
                //.forward(17.0)
                //.lineTo(new Vector2d(-25.0, 40.0),new  LinearInterpolator(Math.toRadians(-110), Math.toRadians(-90.0)))
                //.back(30.0)
                // .reverse()
                //.forward(30.0)
                .build();



        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        if (isStopRequested()) return;

        Trajectory one = drive.trajectoryBuilder()
                .forward(20.0)
                .build();
        Trajectory two = drive.trajectoryBuilder()
                .back(20.0)
                .build();

        drive.followTrajectorySync(test);
//        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
//        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
//        drive.waitForSync(5.0);
//        drive.followTrajectorySync(two);

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
