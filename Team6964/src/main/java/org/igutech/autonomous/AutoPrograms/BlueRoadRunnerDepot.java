package org.igutech.autonomous.AutoPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.util.AutoCVUtil;
import org.igutech.autonomous.util.AutoUtilManager;

import kotlin.Unit;

import static org.igutech.autonomous.roadrunner.MecanumDriveBase.BASE_CONSTRAINTS;

@Config
@Autonomous(name = "BlueRoadRunnerDepot", group = "igutech")
public class BlueRoadRunnerDepot extends LinearOpMode {

    public static AutoUtilManager manager;

    ElapsedTime runtime = new ElapsedTime();
    public static int x = -25;
    public static int y = -10;
    public static int angle = -30;


    public final int TICK_PER_STONE = 335;
    public boolean elevatorRunning = false;
    public boolean reset = true;
    private int level;
    public static int startPos;
    private int setPoint = 0;

    public static AutoCVUtil.Pattern testPattern = AutoCVUtil.Pattern.PATTERN_A;

    IguMecanumDriveBase drive;
    DriveConstraints slowConstraints = new DriveConstraints(
            30, 15, BASE_CONSTRAINTS.maxJerk,
            BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk);

    @Override
    public void runOpMode() {

        manager = new AutoUtilManager(hardwareMap, "BlueRoadRunnerDepot");
        drive = new IguMecanumDriveBase(manager);
        manager.getDriveUtil().resetEncoders();
        drive.setPoseEstimate(new Pose2d(-33.0, 63.0, Math.toRadians(-90.0)));

        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.55);
        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.99);
        manager.getHardware().getServos().get("RotationServo").setPosition(0.2);
        manager.getHardware().getServos().get("CapServo").setPosition(0.29);
        manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);

        manager.getCvUtil().activate();
        AutoCVUtil.Pattern pattern = AutoCVUtil.Pattern.UNKNOWN;

        level = 0;

        while (!opModeIsActive() && !isStopRequested()) {
            AutoCVUtil.Pattern currentPattern = manager.getCvUtil().getPattern();
            if (!currentPattern.equals(AutoCVUtil.Pattern.UNKNOWN))
                pattern = currentPattern;
            telemetry.addData("status", "waiting for start command...");
            telemetry.addData("pattern", pattern);
            telemetry.addData("currentPattern", currentPattern);
            //telemetry.addData("start", startPos);
            telemetry.update();
        }

        if (isStopRequested()) return;
        final AutoCVUtil.Pattern patternFinal = pattern;
        telemetry.addData("Final Pattern", patternFinal);
        telemetry.update();
        new Thread(() -> {
            manager.getCvUtil().shutdown();
        }).start();

        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.OFF);

        if (patternFinal == AutoCVUtil.Pattern.PATTERN_C) {
            Trajectory preIntakePatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.2);
                        return Unit.INSTANCE;
                    })
                    //intake
                    .lineTo(new Vector2d(-33.0, 35.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(20.0)))
                    .build();
            drive.followTrajectorySync(preIntakePatternA);
            manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
            manager.getHardware().getMotors().get("right_intake").setPower(0.6);
            manager.getHardware().getMotors().get("transferMotor").setPower(-1.0);
            Trajectory intakePatternA = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .lineTo(new Vector2d(-33.0, 25.0), new LinearInterpolator(Math.toRadians(-70.0), Math.toRadians(0.0)))
                    .build();
            drive.followTrajectorySync(intakePatternA);
            sleep(400);
            Trajectory moveToFoundationPatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(1.0, () -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.6);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.6);
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.99);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.5, () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);
                        return Unit.INSTANCE;
                    })
                    .addMarker(2.0, () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.99);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(-33.0, 40), new LinearInterpolator(Math.toRadians(-70.0), Math.toRadians(0.0)))
                    .lineTo(new Vector2d(-0.0, 40), new LinearInterpolator(Math.toRadians(-70.0), Math.toRadians(-110.0)))
                    .addMarker(new Vector2d(18, -40), () -> {
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.UP);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(38, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .lineTo(new Vector2d(50, 35), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                    .lineTo(new Vector2d(50, 34), new LinearInterpolator(Math.toRadians(-270), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(moveToFoundationPatternA);
            manager.getHardware().getServos().get("RotationServo").setPosition(0.88);
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.93);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.6);
            sleep(500);
            Trajectory moveFoundationPatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(0.5, () -> {
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.DOWN);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(30, 50), new LinearInterpolator(Math.toRadians(-270), Math.toRadians(30)))
                    .lineTo(new Vector2d(5, 50), new LinearInterpolator(Math.toRadians(-240), Math.toRadians(60)))
                    .lineTo(new Vector2d(5, 35), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(moveFoundationPatternA);
            manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);
            drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.OFF);
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.55);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.99);

            sleep(200);
            drive.setPoseEstimate(new Pose2d(30, 36, Math.toRadians(-180)));

            Trajectory moveToSecondStonePatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(0.5, () -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
                        manager.getHardware().getMotors().get("right_intake").setPower(0.6);
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.OFF);
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.2);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(0, 35), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))

                    .lineTo(new Vector2d(-4, 35), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(moveToSecondStonePatternA);

            drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.OFF);
            Trajectory intakeSecondStone = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .lineTo(new Vector2d(-12, 24), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(10)))
                    .lineTo(new Vector2d(-15, 20), new LinearInterpolator(Math.toRadians(-170), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(intakeSecondStone);

            Trajectory moveToFoundationSecondTimePatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .lineTo(new Vector2d(-30, 45), new LinearInterpolator(Math.toRadians(-170), Math.toRadians(-10)))
                    .addMarker(0.5, () -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.6);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.6);
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.99);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.0, () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.5, () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.99);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(0.0, 45), new LinearInterpolator(Math.toRadians(-180.0), Math.toRadians(0.0)))
                    .addMarker(new Vector2d(18, 45), () -> {
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.UP);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(20, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .lineTo(new Vector2d(65, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(moveToFoundationSecondTimePatternA);
            manager.getHardware().getServos().get("RotationServo").setPosition(0.88);
            sleep(400);
            drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.DOWN);
            sleep(1000);
            manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);

            Trajectory parkPatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(0.5, () -> {
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.OFF);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(5, 35), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(parkPatternA);

        }

        if (patternFinal == AutoCVUtil.Pattern.PATTERN_B) {
            Trajectory preIntakePatternB = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.2);
                        return Unit.INSTANCE;
                    })
                    //intake
                    .lineTo(new Vector2d(-33.0, 35.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(10.0)))
                    .build();
            drive.followTrajectorySync(preIntakePatternB);

            manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
            manager.getHardware().getMotors().get("right_intake").setPower(0.6);
            manager.getHardware().getMotors().get("transferMotor").setPower(-1.0);

            Trajectory intakePatternB = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .lineTo(new Vector2d(-33.0, 25.0), new LinearInterpolator(Math.toRadians(-80.0), Math.toRadians(0.0)))
                    .build();
            drive.followTrajectorySync(intakePatternB);

            sleep(400);
            Trajectory moveToFoundationPatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(1.0, () -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.6);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.6);
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.99);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.5, () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);
                        return Unit.INSTANCE;
                    })
                    .addMarker(2.0, () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.99);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(-33.0, 40), new LinearInterpolator(Math.toRadians(-80.0), Math.toRadians(0.0)))
                    .lineTo(new Vector2d(0.0, 40), new LinearInterpolator(Math.toRadians(-80.0), Math.toRadians(-100.0)))
                    .addMarker(new Vector2d(18, 40), () -> {
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.UP);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(38, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .lineTo(new Vector2d(50, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                    .lineTo(new Vector2d(50, 35), new LinearInterpolator(Math.toRadians(-270), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(moveToFoundationPatternA);
            manager.getHardware().getServos().get("RotationServo").setPosition(0.88);
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.93);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.6);
            sleep(500);
            Trajectory moveFoundationPatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(0.5, () -> {
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.DOWN);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(30, 50), new LinearInterpolator(Math.toRadians(-270), Math.toRadians(30)))
                    .lineTo(new Vector2d(5, 50), new LinearInterpolator(Math.toRadians(-240), Math.toRadians(60)))
                    .lineTo(new Vector2d(5, 35), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(moveFoundationPatternA);
            manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);
            drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.OFF);
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.55);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.99);

            sleep(200);
            drive.setPoseEstimate(new Pose2d(30, 36, Math.toRadians(-180)));

            Trajectory moveToSecondStonePatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(0.5, () -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
                        manager.getHardware().getMotors().get("right_intake").setPower(0.6);
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.OFF);
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.2);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(0, 35), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))

                    .lineTo(new Vector2d(-4, 35), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(moveToSecondStonePatternA);

            drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.OFF);
            Trajectory intakeSecondStone = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .lineTo(new Vector2d(-20, 24), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(10)))
                    .lineTo(new Vector2d(-26, 20), new LinearInterpolator(Math.toRadians(-170), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(intakeSecondStone);

            Trajectory moveToFoundationSecondTimePatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .lineTo(new Vector2d(-30, 45), new LinearInterpolator(Math.toRadians(-170), Math.toRadians(-10)))
                    .addMarker(0.5, () -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.6);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.6);
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.99);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.0, () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.5, () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.99);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(0.0, 45), new LinearInterpolator(Math.toRadians(-180.0), Math.toRadians(0.0)))
                    .lineTo(new Vector2d(20, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .addMarker(() -> {
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.UP);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(65, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(moveToFoundationSecondTimePatternA);
            manager.getHardware().getServos().get("RotationServo").setPosition(0.88);
            sleep(400);
            drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.DOWN);
            sleep(1000);
            manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);

            Trajectory parkPatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(0.5, () -> {
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.OFF);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(5, 35), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(parkPatternA);


        }
        if (patternFinal == AutoCVUtil.Pattern.PATTERN_A) {
            Trajectory preIntakePatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.4);
                        return Unit.INSTANCE;
                    })
                    //intake
                    .lineTo(new Vector2d(-33.0, 35.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(-10.0)))
                    .build();
            drive.followTrajectorySync(preIntakePatternA);

            manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
            manager.getHardware().getMotors().get("right_intake").setPower(0.6);
            manager.getHardware().getMotors().get("transferMotor").setPower(-1.0);

            Trajectory intakePatternC = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .lineTo(new Vector2d(-35.0, 20.0), new LinearInterpolator(Math.toRadians(-100.0), Math.toRadians(0.0)))
                    .build();
            drive.followTrajectorySync(intakePatternC);

            Trajectory moveToFoundationPatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(1.0, () -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.6);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.6);
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.99);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.5, () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);
                        return Unit.INSTANCE;
                    })
                    .addMarker(2.0, () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.99);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(-33.0, 42), new LinearInterpolator(Math.toRadians(-100.0), Math.toRadians(0.0)))
                    .lineTo(new Vector2d(0.0, 42), new LinearInterpolator(Math.toRadians(-100.0), Math.toRadians(-80.0)))
                    .addMarker(new Vector2d(18, 42), () -> {
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.UP);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(38, 42), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .lineTo(new Vector2d(50, 42), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                    .lineTo(new Vector2d(50, 35), new LinearInterpolator(Math.toRadians(-270), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(moveToFoundationPatternA);
            manager.getHardware().getServos().get("RotationServo").setPosition(0.88);
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.93);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.6);
            sleep(800);
            Trajectory moveFoundationPatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(0.5, () -> {
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.DOWN);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(30, 50), new LinearInterpolator(Math.toRadians(-270), Math.toRadians(30)))
                    .lineTo(new Vector2d(5, 50), new LinearInterpolator(Math.toRadians(-240), Math.toRadians(60)))
                    .lineTo(new Vector2d(5, 35), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(moveFoundationPatternA);
            manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);
            drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.OFF);
            manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.55);
            manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.99);

            sleep(200);
            drive.setPoseEstimate(new Pose2d(30, 36, Math.toRadians(-180)));

            Trajectory moveToSecondStonePatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(0.5, () -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
                        manager.getHardware().getMotors().get("right_intake").setPower(0.6);
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.OFF);
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.2);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(0, 35), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))

                    .lineTo(new Vector2d(-4, 35), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(moveToSecondStonePatternA);

            drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.OFF);
            Trajectory intakeSecondStone = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .lineTo(new Vector2d(-26, 24), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(10)))
                    .lineTo(new Vector2d(-30, 20), new LinearInterpolator(Math.toRadians(-170), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(intakeSecondStone);

            Trajectory moveToFoundationSecondTimePatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .lineTo(new Vector2d(-30, 45), new LinearInterpolator(Math.toRadians(-170), Math.toRadians(-10)))
                    .addMarker(0.5, () -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.6);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.6);
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.99);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.0, () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);
                        return Unit.INSTANCE;
                    })
                    .addMarker(1.5, () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.99);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(0.0, 45), new LinearInterpolator(Math.toRadians(-180.0), Math.toRadians(0.0)))
                    .lineTo(new Vector2d(20, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .addMarker(() -> {
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.UP);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(65, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(moveToFoundationSecondTimePatternA);
            manager.getHardware().getServos().get("RotationServo").setPosition(0.88);
            sleep(400);
            drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.DOWN);
            sleep(1000);
            manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);

            Trajectory parkPatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(0.5, () -> {
                        drive.changeElevatorState(IguMecanumDriveBase.ElevatorState.OFF);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(5, 35), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySync(parkPatternA);


        }

        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
        }

    }


}
