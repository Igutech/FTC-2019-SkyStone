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
    public static int strafe = 5;

    public static int forward = 100;


    @Override
    public void runOpMode() throws InterruptedException {

        DriveConstraints slowConstraints = new DriveConstraints(
                35, BASE_CONSTRAINTS.maxAccel, BASE_CONSTRAINTS.maxJerk,
                BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk);

        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "BlueRoadRunnerDepot");
        manager.getDriveUtil().resetEncoders();
        IguMecanumDriveBase drive = new IguMecanumDriveBase(manager);
        drive.setPoseEstimate(new Pose2d(-33.0, 63.0, Math.toRadians(0.0)));

        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
        manager.getHardware().getServos().get("TransferServo").setPosition(0.43);
        manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
        manager.getHardware().getServos().get("CapServo").setPosition(0.53);

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

        if (pattern == AutoCVUtil.Pattern.PATTERN_A) {

        }

        if (pattern == AutoCVUtil.Pattern.PATTERN_B) {

            //get ready for turn
            Trajectory patternB = drive.trajectoryBuilder()
                    .strafeRight(20.0)
                    .forward(17.0)
                    .build();
            drive.followTrajectorySync(patternB);

            drive.turnSync(Math.toRadians(178));

            //intake then drive to foundation
            Trajectory patternB2 = drive.trajectoryBuilder()
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .addMarker(2.5,() -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                        return Unit.INSTANCE;
                    })
                    .addMarker(3.5,() -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
                        return Unit.INSTANCE;
                    })
                    .strafeLeft(28)
                    .forward(5)
                    .strafeRight(28)
                    .back(75)
                    .build();
            drive.followTrajectorySync(patternB2);

            drive.turnSync(Math.toRadians(-90));

            //back up into the foundation
            Trajectory patternB3 = drive.trajectoryBuilder()
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.0);
                        manager.getHardware().getMotors().get("right_intake").setPower(0.0);
                        return Unit.INSTANCE;
                    })
                    .back(6)
                    .build();
            drive.followTrajectorySync(patternB3);

            //latch onto foundation
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
            drive.waitForSync(0.5);

            //move the foundation forward
            Trajectory patternB4 = drive.trajectoryBuilder()
                    .forward(45)
                    .build();
            drive.followTrajectorySync(patternB4);

            drive.turnSync(Math.toRadians(90));

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
            drive.waitForSync(0.2);

            //park
            Trajectory patternB5 = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .back(45)
                    .strafeLeft(8)
                    .forward(75)
                    .build();
            drive.followTrajectorySync(patternB5);
        }

        if (pattern == AutoCVUtil.Pattern.PATTERN_C ) {

            //position to turn
            Trajectory patternC = drive.trajectoryBuilder()
                    .strafeRight(20.0)
                    .forward(10.0)
                    .build();
            drive.followTrajectorySync(patternC);

            drive.turnSync(Math.toRadians(178));

            //Intake and move to foundation
            Trajectory patternC2 = drive.trajectoryBuilder()

                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .forward(3)
                    .addMarker(2.5,() -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                        return Unit.INSTANCE;
                    })
                    .addMarker(3.5,() -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
                        return Unit.INSTANCE;
                    })
                    .strafeLeft(33)
                    .forward(5)
                    .strafeRight(28)
                    .back(80)
                    .build();
            drive.followTrajectorySync(patternC2);

            drive.turnSync(Math.toRadians(-90));

            //line up with the foundation
            Trajectory patternC3 = drive.trajectoryBuilder()
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.0);
                        manager.getHardware().getMotors().get("right_intake").setPower(0.0);
                        return Unit.INSTANCE;
                    })
                    .back(6)
                    .build();
            drive.followTrajectorySync(patternC3);

            //latch on to foundation
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
            drive.waitForSync(0.5);

            //move foundation
            Trajectory patternC4 = drive.trajectoryBuilder()
                    .forward(45)
                    .build();
            drive.followTrajectorySync(patternC4);
            drive.turnSync(Math.toRadians(90));

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
            drive.waitForSync(0.2);

            //park
            Trajectory patternC5 = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .back(45)
                    .strafeLeft(8)
                    .forward(75)
                    .build();
            drive.followTrajectorySync(patternC5);
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







