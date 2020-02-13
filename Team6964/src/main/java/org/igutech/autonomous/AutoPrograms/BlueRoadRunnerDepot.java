package org.igutech.autonomous.AutoPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.roadrunner.MecanumDriveBase;
import org.igutech.autonomous.tuning.DashboardUtil;
import org.igutech.autonomous.util.AutoCVUtil;
import org.igutech.autonomous.util.AutoUtilManager;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import kotlin.Unit;

import static org.igutech.autonomous.roadrunner.MecanumDriveBase.BASE_CONSTRAINTS;

@Config
@Autonomous(name = "BlueRoadRunnerDepot", group = "igutech")
public class BlueRoadRunnerDepot extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public static int x = 45;
    public static int y = 36;
    public static int x2 = 10;
    public static int back = 5;

    @Override
    public void runOpMode() throws InterruptedException {

        DriveConstraints slowConstraints = new DriveConstraints(
                35, 20, BASE_CONSTRAINTS.maxJerk,
                BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk);

        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "BlueRoadRunnerDepot");
        manager.getDriveUtil().resetEncoders();
        IguMecanumDriveBase drive = new IguMecanumDriveBase(manager);
        drive.setPoseEstimate(new Pose2d(-33.0, 63.0, Math.toRadians(-90.0)));
        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
        //manager.getHardware().getServos().get("TransferServo").setPosition(0.43);
        manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
        manager.getHardware().getServos().get("CapServo").setPosition(0.54);

        manager.getCvUtil().activate();
        AutoCVUtil.Pattern pattern = AutoCVUtil.Pattern.UNKNOWN;

        while (!opModeIsActive() && !isStopRequested()) {
            AutoCVUtil.Pattern currentPattern = manager.getCvUtil().getPattern();
            if (!currentPattern.equals(AutoCVUtil.Pattern.UNKNOWN))
                pattern = currentPattern;
            telemetry.addData("status", "waiting for start command...");
            telemetry.addData("pattern", pattern);
            telemetry.addData("currentPattern", currentPattern);
            telemetry.update();
        }

        if (isStopRequested()) return;
        final AutoCVUtil.Pattern patternFinal = pattern;
        telemetry.addData("Final Pattern", patternFinal);
        telemetry.update();
        new Thread(() -> {
            manager.getCvUtil().shutdown();
        }).start();

        /**
         * PATTERN A AND B ARE FLIPPED
         * B IS ACTUAL A AND A IS ACTUAL B
         */
        /** Pattern A
        Trajectory preIntake = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)

                .addMarker(() -> {
                    manager.getHardware().getMotors().get("right_intake").setPower(-0.2);
                    return Unit.INSTANCE;
                })
                //intake
                .lineTo(new Vector2d(-33.0, 35.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(20.0)))
                .build();
        drive.followTrajectorySync(preIntake);
        manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
        manager.getHardware().getMotors().get("right_intake").setPower(0.6);
        manager.getHardware().getMotors().get("transferMotor").setPower(-1.0);
        Trajectory intake = new TrajectoryBuilder(drive.getPoseEstimate(),slowConstraints)
                .lineTo(new Vector2d(-33.0, 25.0), new LinearInterpolator(Math.toRadians(-70.0), Math.toRadians(0.0)))
                .build();
        drive.followTrajectorySync(intake);
        sleep(400);
        Trajectory backup = new TrajectoryBuilder(drive.getPoseEstimate(),BASE_CONSTRAINTS)
                .lineTo(new Vector2d(-33.0, 40), new LinearInterpolator(Math.toRadians(-70.0), Math.toRadians(0.0)))
                .lineTo(new Vector2d(-0.0, 40), new LinearInterpolator(Math.toRadians(-70.0), Math.toRadians(-110.0)))
                .lineTo(new Vector2d(38, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                .lineTo(new Vector2d(50, 33), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                .build();
        drive.followTrajectorySync(backup);
         **/
        /** pattern B
        Trajectory preIntake = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)

                .addMarker(() -> {
                    manager.getHardware().getMotors().get("right_intake").setPower(-0.2);
                    return Unit.INSTANCE;
                })
                //intake
                .lineTo(new Vector2d(-33.0, 35.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(-10.0)))
                .build();
        drive.followTrajectorySync(preIntake);
        manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
        manager.getHardware().getMotors().get("right_intake").setPower(0.6);
        manager.getHardware().getMotors().get("transferMotor").setPower(-1.0);
        Trajectory intake = new TrajectoryBuilder(drive.getPoseEstimate(),slowConstraints)
                .lineTo(new Vector2d(-33.0, 25.0), new LinearInterpolator(Math.toRadians(-100.0), Math.toRadians(0.0)))
                .build();
        drive.followTrajectorySync(intake);
        sleep(400);
        Trajectory backup = new TrajectoryBuilder(drive.getPoseEstimate(),BASE_CONSTRAINTS)
                .lineTo(new Vector2d(-33.0, 40), new LinearInterpolator(Math.toRadians(-100.0), Math.toRadians(0.0)))
                .lineTo(new Vector2d(-0.0, 40), new LinearInterpolator(Math.toRadians(-100), Math.toRadians(-80)))
                .lineTo(new Vector2d(38, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                .lineTo(new Vector2d(50, 33), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                .build();

        drive.followTrajectorySync(backup);
        */

        Trajectory preIntake = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)

                .addMarker(() -> {
                    manager.getHardware().getMotors().get("right_intake").setPower(-0.2);
                    return Unit.INSTANCE;
                })
                //intake
                .lineTo(new  Vector2d(-33.0, 35.0), new  LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(-30.0)))
                .build();
        drive.followTrajectorySync(preIntake);
        manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
        manager.getHardware().getMotors().get("right_intake").setPower(0.6);
        manager.getHardware().getMotors().get("transferMotor").setPower(-1.0);
        Trajectory intake = new TrajectoryBuilder(drive.getPoseEstimate(),slowConstraints)
                .lineTo(new Vector2d(-35.0, 20.0), new LinearInterpolator(Math.toRadians(-120.0), Math.toRadians(-0.0)))
                .build();
        drive.followTrajectorySync(intake);
        sleep(400);
        Trajectory backup = new TrajectoryBuilder(drive.getPoseEstimate(),BASE_CONSTRAINTS)
                .lineTo(new Vector2d(-33.0, 40), new LinearInterpolator(Math.toRadians(-120.0), Math.toRadians(0.0)))
                .lineTo(new Vector2d(-0.0, 40), new LinearInterpolator(Math.toRadians(-120), Math.toRadians(-80)))
                .lineTo(new Vector2d(38, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                .lineTo(new Vector2d(50, 33), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                .build();
        drive.followTrajectorySync(backup);


        //-33.0, 63.0, Math.toRadians(-90.0)

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







