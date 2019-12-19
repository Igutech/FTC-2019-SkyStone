package org.igutech.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.util.AutoUtilManager;

import kotlin.Unit;

@Config
@Autonomous(name="RoadRunnerTestOp", group="igutech")
public class RoadRunnerTestOp extends LinearOpMode {
    public static int back = 15;
    public static int angle = 250;
    public static int strafeLeft  = 45;
    public static int strafeLeft2 = 40;
    public static int x = -90;
    public static int y = 28;
    public static int forward = 40;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {

        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "TestRoadrunner");
        manager.getDriveUtil().resetEncoders();


        IguMecanumDriveBase drive = new IguMecanumDriveBase(manager);

        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
        manager.getHardware().getServos().get("TransferServo").setPosition(0.43);
       // manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);





/*
        manager.getCvUtil().activate();
        AutoCVUtil.Pattern pattern = AutoCVUtil.Pattern.UNKNOWN;*/

        while (!opModeIsActive() && !isStopRequested()) {
         /*   AutoCVUtil.Pattern currentPattern = manager.getCvUtil().getPattern();
            if (!currentPattern.equals(AutoCVUtil.Pattern.UNKNOWN))
                pattern = currentPattern;*/
            telemetry.addData("status", "waiting for start command...");
           /* telemetry.addData("pattern", pattern);
            telemetry.addData("currentPattern", currentPattern);*/
            telemetry.update();

        }

        if (isStopRequested()) return;
        //final AutoCVUtil.Pattern patternFinal = pattern;
        //telemetry.addData("Final Pattern", patternFinal);
        telemetry.update();

     /*   new Thread(() -> {
            manager.getCvUtil().shutdown();
        }).start();
*/
        Trajectory leftTrajectory = drive.trajectoryBuilder()
                .back(3)

                .strafeLeft(45)
                .addMarker(() -> {
                    manager.getHardware().getMotors().get("left_intake").setPower(0.5);
                    manager.getHardware().getMotors().get("right_intake").setPower(-0.5);
                    return Unit.INSTANCE;})
                .forward(13)

                .strafeRight(20)
                .addMarker(()-> {
                    manager.getHardware().getServos().get("GrabberServo").setPosition(0.28);
                    return Unit.INSTANCE;})
                .addMarker(()-> {
                    manager.getHardware().getMotors().get("left_intake").setPower(0.0);
                    manager.getHardware().getMotors().get("right_intake").setPower(0.0);
                    return Unit.INSTANCE;})
               /* .addMarker(()-> {
                    manager.getHardware().getServos().get("GrabberServo").setPosition(0.28);
                    return Unit.INSTANCE;})*/
                .strafeRight(20)

                .addMarker(()-> {
                    manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                    return Unit.INSTANCE;})



                .addMarker(new Vector2d(-30.0,13.0),()->{
                    manager.getHardware().getMotors().get("stoneElevator").setPower(-0.1);
                    return Unit.INSTANCE;})

                // .strafeRight(strafeLeft2)
                .lineTo(new Vector2d(-90,28),new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(-110)))
/*                .addMarker(()->{
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
                    return Unit.INSTANCE;})
                .addMarker(()->{
                    manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
                    return Unit.INSTANCE;})
                .back(5)
                .addMarker(()->{
                    manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
                    return Unit.INSTANCE;})
                .addMarker(()-> {
                    manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
                    manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
                    return Unit.INSTANCE;})
                .back(10)
                .forward(35)*/

                // .back(back)
                .build();
        drive.followTrajectorySync(leftTrajectory);
        drive.turnSync(Math.toRadians(250));

        Trajectory two = drive.trajectoryBuilder()

                .strafeRight(forward)
                .forward(10)
                .addMarker(()-> {
                    manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
                    manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
                    return Unit.INSTANCE;})
                .forward(30)

                .build();

        drive.followTrajectorySync(two);






        while (!isStopRequested() && drive.isBusy()) {

            Pose2d currentPose = drive.getPoseEstimate();
            dashboardTelemetry.addData("backfirst",back);
            dashboardTelemetry.addData("strafe",strafeLeft);
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
