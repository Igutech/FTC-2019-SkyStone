package org.igutech.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.tuning.DashboardUtil;
import org.igutech.autonomous.util.AutoCVUtil;
import org.igutech.autonomous.util.AutoUtilManager;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import kotlin.Unit;

@Config
@Autonomous(name = "BlueRoadRunnerDepot", group = "igutech")
public class BlueRoadRunnerDepot extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static int pattern = 2;

    @Override
    public void runOpMode() throws InterruptedException {

        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "TestRoadrunner");
        manager.getDriveUtil().resetEncoders();
        IguMecanumDriveBase drive = new IguMecanumDriveBase(manager);
        drive.setPoseEstimate(new Pose2d(-33.0, 63.0, Math.toRadians(0.0)));

        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
        manager.getHardware().getServos().get("TransferServo").setPosition(0.43);
        manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
        manager.getHardware().getServos().get("CapServo").setPosition(0.53);

//        manager.getCvUtil().activate();
//        AutoCVUtil.Pattern pattern = AutoCVUtil.Pattern.UNKNOWN;

        while (!opModeIsActive() && !isStopRequested()) {
            //AutoCVUtil.Pattern currentPattern = manager.getCvUtil().getPattern();
//            if (!currentPattern.equals(AutoCVUtil.Pattern.UNKNOWN))
//                pattern = currentPattern;
            telemetry.addData("status", "waiting for start command...");
//            telemetry.addData("pattern", pattern);
//            telemetry.addData("currentPattern", currentPattern);
            telemetry.update();
        }

        if (isStopRequested()) return;
       /* final AutoCVUtil.Pattern patternFinal = pattern;
        telemetry.addData("Final Pattern", patternFinal);
        telemetry.update();
        new Thread(() -> {
            manager.getCvUtil().shutdown();
        }).start();*/

        if (pattern == 1) {
            Trajectory test = drive.trajectoryBuilder()
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .back(10.0)
                    .strafeRight(15)
                    .lineTo(new Vector2d(-50,25), new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(-140)))
                    .lineTo(new Vector2d(-50,15), new LinearInterpolator(Math.toRadians(-140),Math.toRadians(0)))
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
                    .lineTo( new Vector2d(-40.0,40.0), new LinearInterpolator(Math.toRadians(-140.0),Math.toRadians(0.0)))
                    .lineTo( new Vector2d(10.0,37.0), new LinearInterpolator(Math.toRadians(-140.0),Math.toRadians(150.0)))

                    .lineTo(new Vector2d(45.0,40.0),  new LinearInterpolator(Math.toRadians(10.0),Math.toRadians(80.0)))
                    .lineTo( new Vector2d(45.0,22.0),  new LinearInterpolator(Math.toRadians(80.0),Math.toRadians(0.0)))


                    .build();



            drive.followTrajectorySync(test);
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
            drive.waitForSync(1.0);

            Trajectory two = drive.trajectoryBuilder()
                    .forward(35)
                    .build();
            drive.followTrajectorySync(two);
            drive.turnSync(Math.toRadians(350));

            manager.getHardware().getMotors().get("stoneElevator").setPower(0.6);
            sleep(600);
            manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
            sleep(400);
            manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
            sleep(400);

            manager.getHardware().getMotors().get("stoneElevator").setPower(-0.6);
            sleep(600);
            manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);

            manager.getHardware().getMotors().get("stoneElevator").setPower(0.6);
            sleep(600);
            manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
            sleep(400);
            manager.getHardware().getServos().get("RotationServo").setPosition(0.28);
            sleep(600);

            manager.getHardware().getMotors().get("stoneElevator").setPower(-0.6);
            sleep(600);
            manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);



            Trajectory three = drive.trajectoryBuilder()
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .strafeLeft(6)
                    .forward(80)
                    .lineTo(new Vector2d(-65,30), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                    .lineTo(new Vector2d(-65,10), new LinearInterpolator(Math.toRadians(180), Math.toRadians(-10)))

                    .build();
            drive.followTrajectorySync(three);
        }

        if (pattern == 2) {
            Trajectory patternB = drive.trajectoryBuilder()
                    .forward(10.0)
                    .strafeRight(15.0)
                    .lineTo( new Vector2d(-20.0,35.0), new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(-180.0)))
                    .strafeLeft(20)
                    .build();
            drive.followTrajectorySync(patternB);
        }

        if (pattern == 3) {
            Trajectory patternC = drive.trajectoryBuilder()
                    .strafeRight(3.0)
                    .back(20.0)
                    .strafeRight(40.0)
                    .forward(5.0)
                    .strafeLeft(20.0)
                    .lineTo(new Vector2d(50.0, 35.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(90.0)))
                    .build();

            drive.followTrajectorySync(patternC);
        }

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



