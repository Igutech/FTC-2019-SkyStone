package org.igutech.autonomous.AutoPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.util.AutoCVUtil;
import org.igutech.autonomous.util.AutoUtilManager;

import kotlin.Unit;

import static org.igutech.autonomous.roadrunner.MecanumDriveBase.BASE_CONSTRAINTS;

@Config
@Autonomous(name = "RedRoadRunnerDepot", group = "igutech")
public class RedRoadRunnerDepot extends LinearOpMode {
    public static int foward = 3;
    public static int strafe = 14;
    public static int back2 = 15;
    public static double x = 42.5;

    public static int x2 = 36;


    DriveConstraints slowConstraints = new DriveConstraints(
            35, 30, BASE_CONSTRAINTS.maxJerk,
            BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk);

    DriveConstraints fastConstraints = new DriveConstraints(
            45, 40, BASE_CONSTRAINTS.maxJerk,
            BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk);

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {

        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "RedRoadRunnerDepot");
        manager.getDriveUtil().resetEncoders();
        IguMecanumDriveBase drive = new IguMecanumDriveBase(manager);
        drive.setPoseEstimate(new Pose2d(50.0,-63.0,Math.toRadians(-180.0)));

        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
        manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
        manager.getHardware().getServos().get("CapServo").setPosition(0.54 );


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



        while (!isStopRequested() && drive.isBusy()) {

            Pose2d currentPose = drive.getPoseEstimate();
            dashboardTelemetry.addData("x", currentPose.getX());
            dashboardTelemetry.addData("y", currentPose.getY());
            dashboardTelemetry.addData("heading", currentPose.getHeading());
            dashboardTelemetry.addData("x error", drive.getLastError().getX());
            dashboardTelemetry.addData("y error", drive.getLastError().getY());
            dashboardTelemetry.addData("heading error", drive.getLastError().getHeading());
            dashboardTelemetry.update();

//            telemetry.addData("x", currentPose.getX());
//            telemetry.addData("y", currentPose.getY());
//            telemetry.addData("heading", currentPose.getHeading());
//            telemetry.addData("x error", drive.getFollowingError().getX());
//            telemetry.addData("y error", drive.getFollowingError().getY());
//            telemetry.addData("heading error", drive.getFollowingError().getHeading());
//            telemetry.addData("kv",drive.kV);
//
//            telemetry.update();
            //dashboardTelemetry.update();

            drive.update();
        }


    }

}



