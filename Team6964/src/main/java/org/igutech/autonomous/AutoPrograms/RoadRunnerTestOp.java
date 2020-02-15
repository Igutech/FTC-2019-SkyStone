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
@Autonomous(name = "RoadRunnerTestOp", group = "igutech")
public class RoadRunnerTestOp extends LinearOpMode {

    private final int TICK_PER_STONE = 335;
    private final DriveConstraints SLOW_CONSTRAINTS = new DriveConstraints(
            35, 30, BASE_CONSTRAINTS.maxJerk,
            BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk);

    private FtcDashboard dashboard;

    private AutoUtilManager manager;
    public static double p=0.02;
    public static double i=0.0;
    public static double d=0.0002;
    public static int elevatorError;

    private PIDController elevatorController = new PIDController(p, i, d);
    private TrajectoryState state;
    private ElevatorState elevatorState;
    private int level=0;

    IguMecanumDriveBase drive;



    @Override
    public void runOpMode() throws InterruptedException {

        dashboard= FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        manager = new AutoUtilManager(hardwareMap, "RoadRunnerTestOp");
        drive = new IguMecanumDriveBase(manager);
        manager.getDriveUtil().resetEncoders();
        drive.setPoseEstimate(new Pose2d(30, -36, Math.toRadians(180)));


//        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
//        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
//        manager.getHardware().getServos().get("RotationServo").setPosition(0.2);
//        manager.getHardware().getServos().get("CapServo").setPosition(0.54);
//        manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);


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

        Trajectory moveFoundationPatternA = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                .addMarker(0.5,() -> {

                    manager.getHardware().getMotors().get("left_intake").setPower(-0.6);
                    manager.getHardware().getMotors().get("right_intake").setPower(0.6);
                    manager.getHardware().getMotors().get("transferMotor").setPower(-1.0);
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(30, -45), new LinearInterpolator(Math.toRadians(270), Math.toRadians(-30)))
                .lineTo(new Vector2d(5, -45), new LinearInterpolator(Math.toRadians(240), Math.toRadians(-60)))
                .lineTo(new Vector2d(5, -35), new LinearInterpolator(Math.toRadians(180), Math.toRadians(0)))
                .build();
        drive.followTrajectorySync(moveFoundationPatternA);



        while (!isStopRequested()) {


            drive.update();

        }


    }

    public synchronized void changeTrajectoryState(TrajectoryState changeState) {
        state=changeState;
        switch (changeState) {
            case FORWARD:
                Trajectory forward = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                        .forward(50)
                        .addMarker(() -> {
                            changeTrajectoryState(TrajectoryState.BACKWARD);
                            //changeElevatorState(ElevatorState.DOWN);
                            return Unit.INSTANCE;
                        })
                        .build();
                drive.followTrajectory(forward);
                break;
            case BACKWARD:
                Trajectory backward = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                        .back(50)
                        .addMarker(() -> {
                            changeTrajectoryState(TrajectoryState.STRAFE);
                            //changeTrajectoryState(TrajectoryState.OFF);

                            return Unit.INSTANCE;
                        })
                        .build();
                drive.followTrajectory(backward);
                break;
            case STRAFE:
                Trajectory strafe = new TrajectoryBuilder(drive.getPoseEstimate(), BASE_CONSTRAINTS)
                        .strafeRight(35)
                        .addMarker(() -> {
                            changeTrajectoryState(TrajectoryState.OFF);
                            //changeElevatorState(ElevatorState.OFF);
                            return Unit.INSTANCE;
                        })
                        .build();
                drive.followTrajectory(strafe);
            case OFF:

        }

    }

    public synchronized void changeElevatorState(ElevatorState state) {
        switch (state) {
            case DOWN:
                level=0;
                break;
            case UP:
                level=3;
                break;
            case OFF:
                level=0;
        }

    }


    private enum TrajectoryState {
        FORWARD,
        BACKWARD,
        STRAFE,
        OFF
    }

    private enum ElevatorState {
        DOWN,
        UP,
        OFF

    }

}


