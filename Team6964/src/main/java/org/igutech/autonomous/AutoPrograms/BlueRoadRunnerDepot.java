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

import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.util.AutoCVUtil;
import org.igutech.autonomous.util.AutoUtilManager;
import org.igutech.utils.FTCMath;
import org.igutech.utils.control.PIDController;

import kotlin.Unit;

import static org.igutech.autonomous.roadrunner.MecanumDriveBase.BASE_CONSTRAINTS;

@Config
@Autonomous(name = "BlueRoadRunnerDepot", group = "igutech")
public class BlueRoadRunnerDepot extends LinearOpMode {

    private AutoUtilManager manager;
    public int level = 0;
    public static double p = 0.02;
    public static double i = 0.0;
    public static double d = 0.0002;
    private PIDController elevatorController = new PIDController(p, i, d);
    public int TICK_PER_STONE = 335;
    public boolean elevatorRunning = false;
    public boolean reset = true;


    public static AutoCVUtil.Pattern testPattern = AutoCVUtil.Pattern.PATTERN_C;

    IguMecanumDriveBase drive;

    @Override
    public void runOpMode() {
        DriveConstraints slowConstraints = new DriveConstraints(
                35, 20, BASE_CONSTRAINTS.maxJerk,
                BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk);

        manager = new AutoUtilManager(hardwareMap, "BlueRoadRunnerDepot");
        drive = new IguMecanumDriveBase(manager);
        manager.getDriveUtil().resetEncoders();
        drive.setPoseEstimate(new Pose2d(-33.0, 63.0, Math.toRadians(-90.0)));
        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
        manager.getHardware().getServos().get("RotationServo").setPosition(0.2);
        manager.getHardware().getServos().get("CapServo").setPosition(0.54);
        manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);
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
        int startPos = manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition();
        final AutoCVUtil.Pattern patternFinal = testPattern;
        telemetry.addData("Final Pattern", patternFinal);
        telemetry.update();
        new Thread(() -> {
            manager.getCvUtil().shutdown();
        }).start();

        if (patternFinal == AutoCVUtil.Pattern.PATTERN_A) {
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
                    .lineTo(new Vector2d(-33.0, 40), new LinearInterpolator(Math.toRadians(-70.0), Math.toRadians(0.0)))
                    .lineTo(new Vector2d(-0.0, 40), new LinearInterpolator(Math.toRadians(-70.0), Math.toRadians(-110.0)))
                    .lineTo(new Vector2d(38, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .lineTo(new Vector2d(50, 33), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                    .build();
            drive.followTrajectorySync(moveToFoundationPatternA);

            changeElevatorState(ElevatorState.UP);
            while (elevatorRunning && !isStopRequested()) {
                telemetry.addData("time", System.currentTimeMillis() - time);
                int setPoint = startPos - (level * TICK_PER_STONE);
                elevatorController.updateSetpoint(setPoint);
                double power = elevatorController.update(manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition());
                power = FTCMath.clamp(-0.5, 0.5, power);
                manager.getHardware().getMotors().get("stoneElevator").setPower(power);
                if (reset) {
                    time = System.currentTimeMillis();
                    reset = false;
                }
                if (manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition() < -900) {
                    manager.getHardware().getServos().get("RotationServo").setPosition(0.88);
                    sleep(600);
                    changeElevatorState(ElevatorState.DOWN);
                    if (manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition() > startPos - 10) {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);
                        elevatorRunning = false;
                    }
                }

            }
        }

        if (patternFinal == AutoCVUtil.Pattern.PATTERN_B) {
            Trajectory preIntakePatternB = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.2);
                        return Unit.INSTANCE;
                    })
                    //intake
                    .lineTo(new Vector2d(-33.0, 35.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(-10.0)))
                    .build();
            drive.followTrajectorySync(preIntakePatternB);

            manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
            manager.getHardware().getMotors().get("right_intake").setPower(0.6);
            manager.getHardware().getMotors().get("transferMotor").setPower(-1.0);

            Trajectory intakePatternB = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .lineTo(new Vector2d(-33.0, 25.0), new LinearInterpolator(Math.toRadians(-100.0), Math.toRadians(0.0)))
                    .build();
            drive.followTrajectorySync(intakePatternB);

            Trajectory moveToFoundationPatternB = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .lineTo(new Vector2d(-33.0, 40), new LinearInterpolator(Math.toRadians(-100.0), Math.toRadians(0.0)))
                    .lineTo(new Vector2d(-0.0, 40), new LinearInterpolator(Math.toRadians(-100), Math.toRadians(-80)))
                    .lineTo(new Vector2d(38, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .lineTo(new Vector2d(50, 33), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                    .build();
            drive.followTrajectorySync(moveToFoundationPatternB);

            changeElevatorState(ElevatorState.UP);
            while (elevatorRunning && !isStopRequested()) {
                telemetry.addData("time", System.currentTimeMillis() - time);
                int setPoint = startPos - (level * TICK_PER_STONE);
                elevatorController.updateSetpoint(setPoint);
                double power = elevatorController.update(manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition());
                power = FTCMath.clamp(-0.5, 0.5, power);
                manager.getHardware().getMotors().get("stoneElevator").setPower(power);
                if (reset) {
                    time = System.currentTimeMillis();
                    reset = false;
                }
                if (manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition() < -900) {
                    manager.getHardware().getServos().get("RotationServo").setPosition(0.88);
                    sleep(600);
                    changeElevatorState(ElevatorState.DOWN);
                    if (manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition() > startPos - 10) {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);
                        elevatorRunning = false;
                    }
                }

            }
        }
        if (patternFinal == AutoCVUtil.Pattern.PATTERN_C) {
            Trajectory preIntakePatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.2);
                        return Unit.INSTANCE;
                    })
                    //intake
                    .lineTo(new Vector2d(-33.0, 35.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(-30.0)))
                    .build();
            drive.followTrajectorySync(preIntakePatternA);

            manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
            manager.getHardware().getMotors().get("right_intake").setPower(0.6);
            manager.getHardware().getMotors().get("transferMotor").setPower(-1.0);

            Trajectory intakePatternC = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                    .lineTo(new Vector2d(-35.0, 20.0), new LinearInterpolator(Math.toRadians(-120.0), Math.toRadians(-0.0)))
                    .build();
            drive.followTrajectorySync(intakePatternC);

            Trajectory moveToFoundationPatternC = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                    .lineTo(new Vector2d(-33.0, 40), new LinearInterpolator(Math.toRadians(-120.0), Math.toRadians(0.0)))
                    .lineTo(new Vector2d(-0.0, 40), new LinearInterpolator(Math.toRadians(-120), Math.toRadians(-80)))
                    .addMarker(() -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.99);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(38, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                    .lineTo(new Vector2d(50, 33), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                    .build();
            drive.followTrajectorySync(moveToFoundationPatternC);

            changeElevatorState(ElevatorState.UP);
            while (elevatorRunning && !isStopRequested()) {
                telemetry.addData("time", System.currentTimeMillis() - time);
                int setPoint = startPos - (level * TICK_PER_STONE);
                elevatorController.updateSetpoint(setPoint);
                double power = elevatorController.update(manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition());
                power = FTCMath.clamp(-0.5, 0.5, power);
                manager.getHardware().getMotors().get("stoneElevator").setPower(power);
                if (reset) {
                    time = System.currentTimeMillis();
                    reset = false;
                }
                if (manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition() < -900) {
                    manager.getHardware().getServos().get("RotationServo").setPosition(0.88);
                    sleep(600);
                    changeElevatorState(ElevatorState.DOWN);
                    if (manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition() > startPos - 10) {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);
                        elevatorRunning = false;
                    }
                }

            }
        }

        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
        }
    }

    public synchronized void changeElevatorState(ElevatorState state) {
        switch (state) {
            case DOWN:
                level = 0;
                elevatorRunning = true;
                break;
            case UP:
                level = 3;
                elevatorRunning = true;
                break;
            case OFF:
                level = 0;
                elevatorRunning = false;
        }

    }

    private enum ElevatorState {
        DOWN,
        UP,
        OFF

    }

}

