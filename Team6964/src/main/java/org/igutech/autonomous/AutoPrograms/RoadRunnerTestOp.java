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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.util.AutoCVUtil;
import org.igutech.autonomous.util.AutoUtilManager;

import kotlin.DslMarker;
import kotlin.Unit;

import static org.igutech.autonomous.roadrunner.MecanumDriveBase.BASE_CONSTRAINTS;
@Config
@Disabled
@Autonomous(name = "RoadRunnerTestOp", group = "igutech")
public class RoadRunnerTestOp extends LinearOpMode {
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

        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "RoadRunnerTestOp");
        manager.getDriveUtil().resetEncoders();
        IguMecanumDriveBase drive = new IguMecanumDriveBase(manager);
        drive.setPoseEstimate(new Pose2d(50.0,-63.0,Math.toRadians(-180.0)));

        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
        manager.getHardware().getServos().get("TransferServo").setPosition(0.43);
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
        if (pattern == AutoCVUtil.Pattern.PATTERN_A) {
            Trajectory patternAMoveStone=drive.trajectoryBuilder()
                    .forward(3)
                    .strafeRight(48)
                    .build();
            drive.followTrajectorySync(patternAMoveStone);

            Trajectory patternAIntake = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .forward(7)
                    .strafeLeft(25)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0);
                        manager.getHardware().getMotors().get("right_intake").setPower(0);
                        return Unit.INSTANCE;
                    })
                    .build();
            drive.followTrajectorySync(patternAIntake);

            Trajectory patternCMoveToFoundation=drive.trajectoryBuilder()
                    .setReversed(false)
                    //now backing up toward the foundation x is how far back and y is how close to the foundation
                    .addMarker(() -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.5,() -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(120,-38),new LinearInterpolator(Math.toRadians(180.0), Math.toRadians(0.0)))

//                  //turn so the foundation grabbers can latch
                    .lineTo(new Vector2d(125,-38.0),new LinearInterpolator(Math.toRadians(180.0),Math.toRadians(90.0)))
                    .addMarker(new Vector2d(105,38),() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.4);
                        return Unit.INSTANCE;
                    })
                    // back up a bit more for safety
                    .lineTo(new Vector2d(125,-30.0),new LinearInterpolator(Math.toRadians(-90.0),Math.toRadians(0.0)))
                    .build();
            drive.followTrajectorySync(patternCMoveToFoundation);

            manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
            sleep(500);

            Trajectory patternCMoveFoundation = drive.trajectoryBuilder()
                    .addMarker(new Vector2d(125,-35),() -> {
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
                        return Unit.INSTANCE;
                    })
                    //go forward
                    .lineTo(new Vector2d(126,-65), new LinearInterpolator(Math.toRadians(-90),Math.toRadians(0)))
                    //turn
                    .lineTo(new Vector2d(123,-62), new LinearInterpolator(Math.toRadians(-90),Math.toRadians(-90)))
                    .build();
            drive.followTrajectorySync(patternCMoveFoundation);

            manager.getHardware().getMotors().get("stoneElevator").setPower(-0.6);
            sleep(400);
            manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
            manager.getHardware().getServos().get("TransferServo").setPosition(0.43);
            sleep(200);

            Trajectory patternCReleaseStoneAndGoToSecond = drive.trajectoryBuilder()
                    .forward(1)
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
                    .back(15)
                    .strafeRight(20)
                    //to go second stone, -55,-37
                    .lineTo(new Vector2d(x2,-37), new LinearInterpolator(Math.toRadians(180),Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(patternCReleaseStoneAndGoToSecond);

            Trajectory patternCIntakeTwo = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .strafeRight(strafe)
                    .forward(6)
                    .strafeLeft(20)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0);
                        manager.getHardware().getMotors().get("right_intake").setPower(0);
                        return Unit.INSTANCE;
                    })
                    .build();
            drive.followTrajectorySync(patternCIntakeTwo);

            Trajectory patternCPlace = new TrajectoryBuilder(drive.getPoseEstimate(), fastConstraints)
                    .addMarker(0.5,() -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.5,() -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
                        return Unit.INSTANCE;
                    })
                    .strafeLeft(10)
                    .addMarker(new Vector2d(100,-40),() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.6);
                        return Unit.INSTANCE;
                    })
                    .addMarker(new Vector2d(115,-40),() -> {
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
                        return Unit.INSTANCE;
                    })

                    .lineTo(new Vector2d(125,-40),new LinearInterpolator(Math.toRadians(180.0),Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(patternCPlace);

            manager.getHardware().getMotors().get("stoneElevator").setPower(-0.7);
            sleep(600);
            manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);

            manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);

            Trajectory park = new TrajectoryBuilder(drive.getPoseEstimate(), fastConstraints)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(-0.3);
                        return Unit.INSTANCE;
                    })
                    .addMarker(() -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.43);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(75,-37), new LinearInterpolator(Math.toRadians(180),Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(park);




        } else if (pattern == AutoCVUtil.Pattern.PATTERN_B) {
            Trajectory patternBMoveToStone=drive.trajectoryBuilder()
                    .back(10.0)
                    .strafeRight(48)
                    .build();
            drive.followTrajectorySync(patternBMoveToStone);

            Trajectory patternBIntake = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .forward(10)
                    .strafeLeft(25)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0);
                        manager.getHardware().getMotors().get("right_intake").setPower(0);
                        return Unit.INSTANCE;
                    })
                    .build();
            drive.followTrajectorySync(patternBIntake);

            Trajectory patternBMoveToFoundation=drive.trajectoryBuilder()
                    .setReversed(false)
                    //now backing up toward the foundation x is how far back and y is how close to the foundation
                    .addMarker(() -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.5,() -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(120,-38),new LinearInterpolator(Math.toRadians(180.0), Math.toRadians(0.0)))

//                  //turn so the foundation grabbers can latch
                    .lineTo(new Vector2d(125,-38.0),new LinearInterpolator(Math.toRadians(180.0),Math.toRadians(90.0)))
                    .addMarker(new Vector2d(105,38),() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.4);
                        return Unit.INSTANCE;
                    })
                    // back up a bit more for safety
                    .lineTo(new Vector2d(125,-30.0),new LinearInterpolator(Math.toRadians(-90.0),Math.toRadians(0.0)))
                    .build();
            drive.followTrajectorySync(patternBMoveToFoundation);

            manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
            sleep(500);

            Trajectory patternBMoveFoundation = drive.trajectoryBuilder()
                    .addMarker(new Vector2d(125,-35),() -> {
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
                        return Unit.INSTANCE;
                    })
                    //go forward
                    .lineTo(new Vector2d(126,-65), new LinearInterpolator(Math.toRadians(-90),Math.toRadians(0)))
                    //turn
                    .lineTo(new Vector2d(123,-62), new LinearInterpolator(Math.toRadians(-90),Math.toRadians(-90)))
                    .build();
            drive.followTrajectorySync(patternBMoveFoundation);

            manager.getHardware().getMotors().get("stoneElevator").setPower(-0.6);
            sleep(400);
            manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
            manager.getHardware().getServos().get("TransferServo").setPosition(0.43);
            sleep(200);

            Trajectory patternBReleaseStoneAndGoToSecond = drive.trajectoryBuilder()
                    .forward(1)
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
                    .back(15)
                    .strafeRight(20)
                    //to go second stone, -55,-37
                    .lineTo(new Vector2d(x,-37), new LinearInterpolator(Math.toRadians(180),Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(patternBReleaseStoneAndGoToSecond);

            Trajectory patternBIntakeTwo = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .strafeRight(strafe)
                    .forward(5)
                    .strafeLeft(20)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0);
                        manager.getHardware().getMotors().get("right_intake").setPower(0);
                        return Unit.INSTANCE;
                    })

                    .build();
            drive.followTrajectorySync(patternBIntakeTwo);

            Trajectory patternBPlace = new TrajectoryBuilder(drive.getPoseEstimate(), fastConstraints)
                    .addMarker(0.5,() -> {
                         manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                         return Unit.INSTANCE;
                     })
                    .addMarker(1.5,() -> {
                         manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
                         return Unit.INSTANCE;
                    })
                    .strafeLeft(10)
                    .addMarker(new Vector2d(100,-40),() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.6);
                        return Unit.INSTANCE;
                    })
                    .addMarker(new Vector2d(115,-40),() -> {
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
                        return Unit.INSTANCE;
                    })

                    .lineTo(new Vector2d(125,-40),new LinearInterpolator(Math.toRadians(180.0),Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(patternBPlace);

            manager.getHardware().getMotors().get("stoneElevator").setPower(-0.7);
            sleep(600);
            manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);

            manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);

            Trajectory park = new TrajectoryBuilder(drive.getPoseEstimate(), fastConstraints)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(-0.3);
                        return Unit.INSTANCE;
                    })
                    .addMarker(() -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.43);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(75,-37), new LinearInterpolator(Math.toRadians(180),Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(park);

        } else if (pattern == AutoCVUtil.Pattern.PATTERN_C ) {

            Trajectory patternCMoveToStone=drive.trajectoryBuilder()
                    .back(3)
                    .strafeRight(47)
                    .build();
            drive.followTrajectorySync(patternCMoveToStone);

            Trajectory patternCIntake = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .forward(6.5)
                    .strafeLeft(25)
                    .back(3)
                    .build();
            drive.followTrajectorySync(patternCIntake);
            manager.getHardware().getMotors().get("left_intake").setPower(0.0);
            manager.getHardware().getMotors().get("right_intake").setPower(0.0);

            Trajectory patternCMoveToFoundation=drive.trajectoryBuilder()
                    .setReversed(false)
                    //now backing up toward the foundation x is how far back and y is how close to the foundation
                    .addMarker(() -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.5,() -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(120,-38),new LinearInterpolator(Math.toRadians(180.0), Math.toRadians(0.0)))

//                  //turn so the foundation grabbers can latch
                    .lineTo(new Vector2d(125,-38.0),new LinearInterpolator(Math.toRadians(180.0),Math.toRadians(90.0)))
                    .addMarker(new Vector2d(105,38),() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.4);
                        return Unit.INSTANCE;
                    })
                    // back up a bit more for safety
                    .lineTo(new Vector2d(125,-30.0),new LinearInterpolator(Math.toRadians(-90.0),Math.toRadians(0.0)))
                    .build();
            drive.followTrajectorySync(patternCMoveToFoundation);

            manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
            sleep(500);

            Trajectory patternCMoveFoundation = drive.trajectoryBuilder()
                    .addMarker(new Vector2d(125,-35),() -> {
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
                        return Unit.INSTANCE;
                    })
                    //go forward
                    .lineTo(new Vector2d(126,-65), new LinearInterpolator(Math.toRadians(-90),Math.toRadians(0)))
                    //turn
                    .lineTo(new Vector2d(123,-62), new LinearInterpolator(Math.toRadians(-90),Math.toRadians(-90)))
                    .build();
            drive.followTrajectorySync(patternCMoveFoundation);

            manager.getHardware().getMotors().get("stoneElevator").setPower(-0.6);
            sleep(400);
            manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
            manager.getHardware().getServos().get("TransferServo").setPosition(0.43);
            sleep(200);

            Trajectory patternCReleaseStoneAndGoToSecond = drive.trajectoryBuilder()
                    .forward(1)
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
                    .back(15)
                    .strafeRight(20)
                    //to go second stone, -55,-37
                    .lineTo(new Vector2d(x2,-37), new LinearInterpolator(Math.toRadians(180),Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(patternCReleaseStoneAndGoToSecond);

            Trajectory patternCIntakeTwo = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .strafeRight(strafe)
                    .forward(6)
                    .strafeLeft(20)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0);
                        manager.getHardware().getMotors().get("right_intake").setPower(0);
                        return Unit.INSTANCE;
                    })
                    .build();
            drive.followTrajectorySync(patternCIntakeTwo);

            Trajectory patternCPlace = new TrajectoryBuilder(drive.getPoseEstimate(), fastConstraints)
                    .addMarker(0.5,() -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.5,() -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
                        return Unit.INSTANCE;
                    })
                    .strafeLeft(10)
                    .addMarker(new Vector2d(100,-40),() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.6);
                        return Unit.INSTANCE;
                    })
                    .addMarker(new Vector2d(115,-40),() -> {
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
                        return Unit.INSTANCE;
                    })

                    .lineTo(new Vector2d(125,-40),new LinearInterpolator(Math.toRadians(180.0),Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(patternCPlace);

            manager.getHardware().getMotors().get("stoneElevator").setPower(-0.7);
            sleep(600);
            manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);

            manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);

            Trajectory park = new TrajectoryBuilder(drive.getPoseEstimate(), fastConstraints)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(-0.3);
                        return Unit.INSTANCE;
                    })
                    .addMarker(() -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.43);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(75,-37), new LinearInterpolator(Math.toRadians(180),Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(park);



        } else {

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



