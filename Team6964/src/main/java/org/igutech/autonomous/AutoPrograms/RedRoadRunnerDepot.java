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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.util.AutoCVUtil;
import org.igutech.autonomous.util.AutoUtilManager;
import org.igutech.utils.FTCMath;
import org.igutech.utils.control.PIDController;

import kotlin.Unit;

import static org.igutech.autonomous.roadrunner.MecanumDriveBase.BASE_CONSTRAINTS;

@Config
@Autonomous(name = "RedRoadRunnerDepot", group = "igutech")
public class RedRoadRunnerDepot extends LinearOpMode {

    private final int TICK_PER_STONE = 120;
    private final DriveConstraints SLOW_CONSTRAINTS = new DriveConstraints(
            35, 30, BASE_CONSTRAINTS.maxJerk,
            BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk);

    private FtcDashboard dashboard;

    private AutoUtilManager manager;

    private PIDController elevatorController = new PIDController(0.02, 0.0, 0.0);
    private TrajectoryState state;
    private ElevatorState elevatorState;
    private int level=0;

    IguMecanumDriveBase drive;



    @Override
    public void runOpMode() throws InterruptedException {

        dashboard= FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        manager = new AutoUtilManager(hardwareMap, "RedRoadRunnerDepot");
        drive = new IguMecanumDriveBase(manager);
        manager.getDriveUtil().resetEncoders();
        drive.setPoseEstimate(new Pose2d(50.0, -63.0, Math.toRadians(-180.0)));


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

        changeTrajectoryState(TrajectoryState.PREPARE_TO_INTAKE);
        changeElevatorState(ElevatorState.OFF);
        while (!isStopRequested()) {


            int setPoint = level * TICK_PER_STONE;
            elevatorController.updateSetpoint(setPoint);
            if (Math.abs(setPoint - manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition()) > 50) {
                elevatorController.reset(manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition());
            }
            double power = elevatorController.update(manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition());
            power = FTCMath.clamp(-0.5, 0.5, power);
            manager.getHardware().getMotors().get("stoneElevator").setPower(power);

            Pose2d currentPose = drive.getPoseEstimate();
            dashboardTelemetry.addData("x", currentPose.getX());
            dashboardTelemetry.addData("y", currentPose.getY());
            dashboardTelemetry.addData("heading", currentPose.getHeading());
            dashboardTelemetry.addData("x error", drive.getLastError().getX());
            dashboardTelemetry.addData("y error", drive.getLastError().getY());
            dashboardTelemetry.addData("heading error", drive.getLastError().getHeading());
            drive.update();
            dashboardTelemetry.update();

        }


    }

    public synchronized void changeTrajectoryState(TrajectoryState changeState) {
        state=changeState;
        switch (changeState) {

            case PREPARE_TO_INTAKE:
                Trajectory patternBPrepareToIntake = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                        .back(13.0)
                        .strafeRight(42.0)
                        .addMarker(() -> {
                            changeTrajectoryState(TrajectoryState.INTAKE);
                            return Unit.INSTANCE;
                        })
                        .build();
                drive.followTrajectory(patternBPrepareToIntake);
                break;

            case INTAKE:
                Trajectory patternBIntake = new TrajectoryBuilder(drive.getFollower().getTrajectory().end(), SLOW_CONSTRAINTS)
                        .lineTo(new Vector2d(-28.0, -21.0), new LinearInterpolator(Math.toRadians(180.0), Math.toRadians(0.0)))
                        //strafe right
                        .lineTo(new Vector2d(-28.0, -40.0), new LinearInterpolator(Math.toRadians(180.0), Math.toRadians(0.0)))
                        .addMarker(() -> {
                           changeTrajectoryState(TrajectoryState.MOVE_TO_FOUNDATION);
                            return Unit.INSTANCE;
                        })
                        .build();
                drive.followTrajectory(patternBIntake);
                break;

            case MOVE_TO_FOUNDATION:
                Trajectory patternBMoveToFoundation = new TrajectoryBuilder(drive.getFollower().getTrajectory().end(), BASE_CONSTRAINTS)
                        .lineTo(new Vector2d(30.0, -40.0), new LinearInterpolator(Math.toRadians(180.0), Math.toRadians(0.0)))
                        //turns
                        .lineTo(new Vector2d(50.0, -30.0), new LinearInterpolator(Math.toRadians(180.0), Math.toRadians(90.0)))
                        .addMarker(() -> {
                            changeTrajectoryState(TrajectoryState.MOVE_TO_SECOND_STONE);
                            return Unit.INSTANCE;
                        })
                        .build();
                drive.followTrajectory(patternBMoveToFoundation);

                break;

            case MOVE_TO_SECOND_STONE:
                Trajectory patternBMoveToSecondStone = new TrajectoryBuilder(drive.getFollower().getTrajectory().end(), BASE_CONSTRAINTS)
                        .lineTo(new Vector2d(50.0, -40.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(0.0)))
                        //strafe right
                        .lineTo(new Vector2d(-30.0, -40.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(0.0)))
                        //turn for second intake
                        .lineTo(new Vector2d(-50.0, -21.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(-90.0)))
                        .addMarker(() -> {
                            changeTrajectoryState(TrajectoryState.INTAKE_SECOND_STONE);
                            return Unit.INSTANCE;
                        })
                        .build();
                break;

        }

    }

    public synchronized void changeElevatorState(ElevatorState state) {
        switch (state) {
            case WAITING:
                manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
                manager.getHardware().getServos().get("RotationServo").setPosition(0.28);
                level=0;
                break;
            case DOWN:
                level=0;
                break;
            case UP:
                level=3;
                break;
            case PREPARE_TO_STACK:
                manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
                level=1;
                break;
            case STACKING:
               manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
               break;
        }


    }


    private enum TrajectoryState {
        PREPARE_TO_INTAKE,
        INTAKE,
        MOVE_TO_FOUNDATION,
        STACKING,
        MOVE_TO_SECOND_STONE,
        INTAKE_SECOND_STONE,
        MOVE_FOUNDATION,
        PARK
    }

    private enum ElevatorState {
        WAITING,
        UP,
        RESET,
        PREPARE_TO_STACK,
        STACKING,
        DOWN,
        OFF

    }

}



