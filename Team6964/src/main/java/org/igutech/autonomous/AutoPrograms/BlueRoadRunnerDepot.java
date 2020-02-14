package org.igutech.autonomous.AutoPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    private final int TICK_PER_STONE = 335;
    private final DriveConstraints SLOW_CONSTRAINTS = new DriveConstraints(
            35, 30, BASE_CONSTRAINTS.maxJerk,
            BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk);

    private FtcDashboard dashboard;

    private AutoUtilManager manager;
    public static double p = 0.02;
    public static double i = 0.0;
    public static double d = 0.0002;
    public static int elevatorError;

    private PIDController elevatorController = new PIDController(p, i, d);
    private TrajectoryState state;
    private ElevatorState elevatorState;
    private int level = 0;
    private boolean elevatorEnabled = true;

    public static AutoCVUtil.Pattern testPattern= AutoCVUtil.Pattern.PATTERN_C;

    IguMecanumDriveBase drive;
    DriveConstraints slowConstraints = new DriveConstraints(
            35, 20, BASE_CONSTRAINTS.maxJerk,
            BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk);

    private ElapsedTime runtime = new ElapsedTime();
    public boolean reset = true;
    public double time;
    AutoCVUtil.Pattern patternFinal;


    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


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
        //patternFinal = pattern;
        patternFinal=testPattern;
        telemetry.addData("Final Pattern", patternFinal);
        telemetry.update();

        new Thread(() -> {
            manager.getCvUtil().shutdown();
        }).start();

        changeTrajectoryState(TrajectoryState.PREPARE_TO_INTAKE);
        changeElevatorState(ElevatorState.OFF);
        while (!isStopRequested()) {

            int setPoint = level * TICK_PER_STONE;
            elevatorController.updateSetpoint(setPoint);
            elevatorError = setPoint - manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition();
            if (Math.abs(setPoint - manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition()) > 50) {
                elevatorController.reset(manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition());
            }
            double power = elevatorController.update(manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition());
            power = FTCMath.clamp(-0.5, 0.5, power);
            if (elevatorEnabled) {
                manager.getHardware().getMotors().get("stoneElevator").setPower(power);
            }
            Pose2d currentPose = drive.getPoseEstimate();
            dashboardTelemetry.addData("x", currentPose.getX());
            dashboardTelemetry.addData("y", currentPose.getY());
            dashboardTelemetry.addData("heading", currentPose.getHeading());
            dashboardTelemetry.addData("x error", drive.getLastError().getX());
            dashboardTelemetry.addData("y error", drive.getLastError().getY());
            dashboardTelemetry.addData("heading error", drive.getLastError().getHeading());
            telemetry.addData("stage", state);
            telemetry.update();
            drive.update();
            dashboardTelemetry.update();

        }
        changeTrajectoryState(TrajectoryState.OFF);
        changeElevatorState(ElevatorState.OFF);
    }

    public synchronized void changeTrajectoryState(TrajectoryState changeState) {
        state = changeState;
        switch (changeState) {
            case PREPARE_TO_INTAKE:
                if (patternFinal == AutoCVUtil.Pattern.PATTERN_A) {
                    Trajectory preIntake = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                            .addMarker(() -> {
                                manager.getHardware().getMotors().get("right_intake").setPower(-0.2);
                                return Unit.INSTANCE;
                            })
                            //intake
                            .lineTo(new Vector2d(-33.0, 35.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(20.0)))
                            .addMarker(() -> {
                                changeTrajectoryState(TrajectoryState.INTAKE_FIRST_STONE);
                                return Unit.INSTANCE;
                            })
                            .build();
                    drive.followTrajectory(preIntake);
                } else if (patternFinal == AutoCVUtil.Pattern.PATTERN_B) {
                    Trajectory preIntake = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                            .addMarker(() -> {
                                manager.getHardware().getMotors().get("right_intake").setPower(-0.2);
                                return Unit.INSTANCE;
                            })
                            //intake
                            .lineTo(new Vector2d(-33.0, 35.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(-10.0)))
                            .addMarker(() -> {
                                changeTrajectoryState(TrajectoryState.INTAKE_FIRST_STONE);
                                return Unit.INSTANCE;
                            })
                            .build();
                    drive.followTrajectory(preIntake);
                } else if (patternFinal == AutoCVUtil.Pattern.PATTERN_C) {
                    Trajectory preIntake = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                            .addMarker(() -> {
                                manager.getHardware().getMotors().get("right_intake").setPower(-0.2);
                                return Unit.INSTANCE;
                            })
                            //intake
                            .lineTo(new Vector2d(-33.0, 35.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(-30.0)))
                            .addMarker(() -> {
                                changeTrajectoryState(TrajectoryState.INTAKE_FIRST_STONE);
                                return Unit.INSTANCE;
                            })
                            .build();
                    drive.followTrajectory(preIntake);
                }
                break;

            case INTAKE_FIRST_STONE:
                if (patternFinal == AutoCVUtil.Pattern.PATTERN_A) {
                    Trajectory intake = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                            .addMarker(() -> {
                                manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
                                manager.getHardware().getMotors().get("right_intake").setPower(0.6);
                                manager.getHardware().getMotors().get("transferMotor").setPower(-1.0);
                                return Unit.INSTANCE;
                            })
                            .lineTo(new Vector2d(-33.0, 25.0), new LinearInterpolator(Math.toRadians(-70.0), Math.toRadians(0.0)))
                            .addMarker(() -> {
                                changeTrajectoryState(TrajectoryState.MOVE_TO_FOUNDATION);
                                return Unit.INSTANCE;
                            })
                            .build();
                    drive.followTrajectory(intake);
                } else if (patternFinal == AutoCVUtil.Pattern.PATTERN_B) {
                    Trajectory intake = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                            .addMarker(() -> {
                                manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
                                manager.getHardware().getMotors().get("right_intake").setPower(0.6);
                                manager.getHardware().getMotors().get("transferMotor").setPower(-1.0);
                                return Unit.INSTANCE;
                            })
                            .lineTo(new Vector2d(-33.0, 25.0), new LinearInterpolator(Math.toRadians(-100.0), Math.toRadians(0.0)))
                            .addMarker(() -> {
                                changeTrajectoryState(TrajectoryState.MOVE_TO_FOUNDATION);
                                return Unit.INSTANCE;
                            })
                            .build();
                    drive.followTrajectory(intake);
                } else if (patternFinal == AutoCVUtil.Pattern.PATTERN_C) {
                    Trajectory intake = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                            .addMarker(() -> {
                                manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
                                manager.getHardware().getMotors().get("right_intake").setPower(0.6);
                                manager.getHardware().getMotors().get("transferMotor").setPower(-1.0);
                                return Unit.INSTANCE;
                            })
                            .lineTo(new Vector2d(-35.0, 20.0), new LinearInterpolator(Math.toRadians(-120.0), Math.toRadians(-0.0)))
                            .addMarker(() -> {
                                changeTrajectoryState(TrajectoryState.MOVE_TO_FOUNDATION);
                                return Unit.INSTANCE;
                            })
                            .build();
                    drive.followTrajectory(intake);
                }
                break;

            case MOVE_TO_FOUNDATION:
                if (patternFinal == AutoCVUtil.Pattern.PATTERN_A) {
                    Trajectory backup = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                            .lineTo(new Vector2d(-33.0, 40), new LinearInterpolator(Math.toRadians(-70.0), Math.toRadians(0.0)))
                            .lineTo(new Vector2d(-0.0, 40), new LinearInterpolator(Math.toRadians(-70.0), Math.toRadians(-110.0)))
                            .lineTo(new Vector2d(38, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                            .lineTo(new Vector2d(50, 33), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                            .addMarker(() -> {
                                changeTrajectoryState(TrajectoryState.LATCH_ON_FOUNDATION_AND_PLACE);
                                return Unit.INSTANCE;
                            })
                            .build();
                    drive.followTrajectory(backup);
                }else if (patternFinal== AutoCVUtil.Pattern.PATTERN_B){
                    Trajectory backup = new TrajectoryBuilder(drive.getPoseEstimate(),BASE_CONSTRAINTS)
                            .lineTo(new Vector2d(-33.0, 40), new LinearInterpolator(Math.toRadians(-100.0), Math.toRadians(0.0)))
                            .lineTo(new Vector2d(-0.0, 40), new LinearInterpolator(Math.toRadians(-100), Math.toRadians(-80)))
                            .lineTo(new Vector2d(38, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                            .lineTo(new Vector2d(50, 33), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                            .addMarker(() -> {
                                changeTrajectoryState(TrajectoryState.LATCH_ON_FOUNDATION_AND_PLACE);
                                return Unit.INSTANCE;
                            })
                            .build();
                    drive.followTrajectory(backup);
                }else if (patternFinal== AutoCVUtil.Pattern.PATTERN_C){
                    Trajectory backup = new TrajectoryBuilder(drive.getPoseEstimate(),BASE_CONSTRAINTS)
                            .lineTo(new Vector2d(-33.0, 40), new LinearInterpolator(Math.toRadians(-120.0), Math.toRadians(0.0)))
                            .lineTo(new Vector2d(-0.0, 40), new LinearInterpolator(Math.toRadians(-120), Math.toRadians(-80)))
                            .lineTo(new Vector2d(38, 40), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(0)))
                            .lineTo(new Vector2d(50, 33), new LinearInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                            .addMarker(() -> {
                                changeTrajectoryState(TrajectoryState.LATCH_ON_FOUNDATION_AND_PLACE);
                                return Unit.INSTANCE;
                            })
                            .build();
                    drive.followTrajectory(backup);
                }

                break;

            case LATCH_ON_FOUNDATION_AND_PLACE:
                break;
            case MOVE_FOUNDATION_AND_GET_SECOND_STONE:

            case OFF:
                elevatorEnabled = false;
                break;

        }

    }

    public synchronized void changeElevatorState(ElevatorState state) {
        switch (state) {
            case DOWN:
                level = 0;
                break;
            case UP:
                level = 5;
                break;
            case OFF:
                elevatorEnabled = false;
                break;
            case DEFAULT:
                if (manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition() < 800) {
                    level = 5;
                }
                if (reset) {
                    time = System.currentTimeMillis();
                    reset = false;
                }

                if (manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition() > 360) {
                    manager.getHardware().getServos().get("RotationServo").setPosition(0.28);

                    if ((System.currentTimeMillis() - time) > 2000) {
                        elevatorState = ElevatorState.DOWN;
                        reset = true;
                    }
                }
                break;
        }

    }


    private enum TrajectoryState {
        PREPARE_TO_INTAKE,
        INTAKE_FIRST_STONE,
        MOVE_TO_FOUNDATION,
        LATCH_ON_FOUNDATION_AND_PLACE,
        MOVE_FOUNDATION_AND_GET_SECOND_STONE,
        OFF

    }

    private enum ElevatorState {
        DOWN,
        UP,
        OFF,
        DEFAULT

    }

}