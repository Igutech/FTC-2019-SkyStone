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

    @Override
    public void runOpMode() throws InterruptedException {

        DriveConstraints slowConstraints = new DriveConstraints(
                35, 20, BASE_CONSTRAINTS.maxJerk,
                BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk);

        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "BlueRoadRunnerDepot");
        manager.getDriveUtil().resetEncoders();
        IguMecanumDriveBase drive = new IguMecanumDriveBase(manager);
        drive.setPoseEstimate(new Pose2d(-33.0, 63.0, Math.toRadians(0.0)));

        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
        manager.getHardware().getServos().get("TransferServo").setPosition(0.43);
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
        if (pattern == AutoCVUtil.Pattern.PATTERN_A) {
            Trajectory patternA = drive.trajectoryBuilder()
                    .back(16)
                    .strafeRight(48)
                    .build();
            drive.followTrajectorySync(patternA);

            Trajectory patternAIntake = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .forward(12)
                    .strafeLeft(28)

                    .build();
            drive.followTrajectorySync(patternAIntake);
            manager.getHardware().getMotors().get("left_intake").setPower(0);
            manager.getHardware().getMotors().get("right_intake").setPower(0);

            Trajectory patternAToFoundation = drive.trajectoryBuilder()
                    //                   .forward(75)
                    .addMarker(1.0,() -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                        return Unit.INSTANCE;
                    })
                    .addMarker(2.5,() -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(x,y),new LinearInterpolator(Math.toRadians(0),Math.toRadians(0)))
                    .lineTo(new Vector2d(x,38),new LinearInterpolator(Math.toRadians(0),Math.toRadians(90)))
                    .lineTo(new Vector2d(x,21),new LinearInterpolator(Math.toRadians(90),Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(patternAToFoundation);

            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
            sleep(400);

            //move foundation
            Trajectory patternAMoveFoundation = drive.trajectoryBuilder()
                    .forward(35)
                    .build();
            drive.followTrajectorySync(patternAMoveFoundation);
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
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
            sleep(200);

            //park
            Trajectory patternAReleaseStoneAndPark = drive.trajectoryBuilder()
                    .forward(2)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.6);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.2,() -> {
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.28);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.6,() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.-6);
                        return Unit.INSTANCE;
                    })
                    .addMarker(2.4,() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0);
                        return Unit.INSTANCE;
                    })
                    .back(20)
                    .strafeLeft(17)
                    .lineTo(new Vector2d(x2,50), new LinearInterpolator(Math.toRadians(180),Math.toRadians(0)))
                    //.forward(75)
                    .build();
            drive.followTrajectorySync(patternAReleaseStoneAndPark);

        }

        if (pattern == AutoCVUtil.Pattern.PATTERN_B) {
            Trajectory patternB = drive.trajectoryBuilder()
                    .back(9)
                    .strafeRight(48)
                    .build();
            drive.followTrajectorySync(patternB);

            Trajectory patternBIntake = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .forward(12)
                    .strafeLeft(28)

                    .build();
            drive.followTrajectorySync(patternBIntake);
            manager.getHardware().getMotors().get("left_intake").setPower(0);
            manager.getHardware().getMotors().get("right_intake").setPower(0);

            Trajectory patternBToFoundation = drive.trajectoryBuilder()
 //                   .forward(75)
                    .addMarker(1.0,() -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                        return Unit.INSTANCE;
                    })
                    .addMarker(2.5,() -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(x,y),new LinearInterpolator(Math.toRadians(0),Math.toRadians(0)))
                    .lineTo(new Vector2d(x,38),new LinearInterpolator(Math.toRadians(0),Math.toRadians(90)))
                    .lineTo(new Vector2d(x,21),new LinearInterpolator(Math.toRadians(90),Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(patternBToFoundation);

            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
            sleep(400);

            //move foundation
            Trajectory patternBMoveFoundation = drive.trajectoryBuilder()
                    .forward(35)
                    .build();
            drive.followTrajectorySync(patternBMoveFoundation);
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
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
            sleep(200);



            //park
            Trajectory patternBReleaseStoneAndPark = drive.trajectoryBuilder()
                    .forward(2)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.6);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.2,() -> {
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.28);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.6,() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.-6);
                        return Unit.INSTANCE;
                    })
                    .addMarker(2.4,() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0);
                        return Unit.INSTANCE;
                    })
                    .back(20)
                    .strafeLeft(17)
                    .lineTo(new Vector2d(x2,50), new LinearInterpolator(Math.toRadians(180),Math.toRadians(0)))
                    //.forward(75)
                    .build();
            drive.followTrajectorySync(patternBReleaseStoneAndPark);

        }

        if (pattern == AutoCVUtil.Pattern.PATTERN_C ) {
            Trajectory patternC = drive.trajectoryBuilder()
                    .back(23)
                    .strafeRight(48)
                    .build();
            drive.followTrajectorySync(patternC);

            Trajectory patternCIntake = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .forward(13)
                    .strafeLeft(28)

                    .build();
            drive.followTrajectorySync(patternCIntake);
            manager.getHardware().getMotors().get("left_intake").setPower(0);
            manager.getHardware().getMotors().get("right_intake").setPower(0);

            Trajectory patternCToFoundation = drive.trajectoryBuilder()
                    //                   .forward(75)
                    .addMarker(1.0,() -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                        return Unit.INSTANCE;
                    })
                    .addMarker(2.5,() -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(x,y),new LinearInterpolator(Math.toRadians(0),Math.toRadians(0)))
                    .lineTo(new Vector2d(x,38),new LinearInterpolator(Math.toRadians(0),Math.toRadians(90)))
                    .lineTo(new Vector2d(x,21),new LinearInterpolator(Math.toRadians(90),Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(patternCToFoundation);

            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
            sleep(400);

            //move foundation
            Trajectory patternCMoveFoundation = drive.trajectoryBuilder()
                    .forward(35)
                    .build();
            drive.followTrajectorySync(patternCMoveFoundation);
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
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
            sleep(200);

            //park
            Trajectory patternCReleaseStoneAndPark = drive.trajectoryBuilder()
                    .forward(2)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.6);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.2,() -> {
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.28);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.6,() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.-6);
                        return Unit.INSTANCE;
                    })
                    .addMarker(2.4,() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0);
                        return Unit.INSTANCE;
                    })
                    .back(20)
                    .strafeLeft(17)
                    .lineTo(new Vector2d(x2,50), new LinearInterpolator(Math.toRadians(180),Math.toRadians(0)))
                    //.forward(75)
                    .build();
            drive.followTrajectorySync(patternCReleaseStoneAndPark);

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







