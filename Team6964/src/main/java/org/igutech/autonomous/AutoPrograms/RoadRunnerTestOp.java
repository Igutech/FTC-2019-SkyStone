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
@Autonomous(name = "RoadRunnerTestOp", group = "igutech")
public class RoadRunnerTestOp extends LinearOpMode {

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
    DriveConstraints fastConstraints = new DriveConstraints(
            50, 30, BASE_CONSTRAINTS.maxJerk,
            Math.toRadians(90), Math.toRadians(90), BASE_CONSTRAINTS.maxAngJerk);

    @Override
    public void runOpMode() {

        manager = new AutoUtilManager(hardwareMap, "RoadRunnerTestOp");
        drive = new IguMecanumDriveBase(manager);
        manager.getDriveUtil().resetEncoders();
        drive.setPoseEstimate(new Pose2d(-33.0, 63.0, Math.toRadians(0)));

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

        Trajectory PatternA = drive.trajectoryBuilder()
                .strafeTo(new Vector2d(-33.0, 33.0))
                .lineTo(new Vector2d(0.0, 37.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(0.0)))
                .lineTo(new Vector2d(50.0, 37.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(0.0)))
                .build();
        drive.followTrajectorySync(PatternA);

        Trajectory one = drive.trajectoryBuilder()
                .lineTo(new Vector2d(0.0, 44.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(0.0)))
                .lineTo(new Vector2d(-50.0, 44.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(0.0)))
                .build();
        drive.followTrajectorySync(one);

        Trajectory two = drive.trajectoryBuilder()
                .lineTo(new Vector2d(0.0, 44.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(0.0)))
                .lineTo(new Vector2d(50.0, 30.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(0.0)))
                .build();
        drive.followTrajectorySync(two);

        Trajectory three = drive.trajectoryBuilder()
                .lineTo(new Vector2d(0.0, 40.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(0.0)))
                .lineTo(new Vector2d(-45.0, 35.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(0.0)))
                .build();
        drive.followTrajectorySync(three);

        Trajectory four = drive.trajectoryBuilder()
                .lineTo(new Vector2d(0, 45.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(0.0)))
                .lineTo(new Vector2d(55.0, 45.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(90.0)))
                .lineTo(new Vector2d(55.0, 25.0), new LinearInterpolator(Math.toRadians(90.0), Math.toRadians(0.0)))
                .build();
        drive.followTrajectorySync(four);
        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.93);
        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.6);
        sleep(500);

        Trajectory five = drive.trajectoryBuilder()
                .lineTo(new Vector2d(55.0, 55), new LinearInterpolator(Math.toRadians(90.0), Math.toRadians(0.0)))
                .build();
        drive.followTrajectorySync(five);
        drive.turnSync(Math.toRadians(90.0));


        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
        }

    }


}
