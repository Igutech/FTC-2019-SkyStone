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
import org.igutech.autonomous.util.AutoCVUtil;
import org.igutech.autonomous.util.AutoUtilManager;

import kotlin.Unit;

@Config
@Autonomous(name = "BlueRoadRunnerDepot", group = "igutech")
public class BlueRoadRunnerDepot extends LinearOpMode {
    public static int strafeLeft2 = 53;
    public static int test = 3;
    //public static int x = 70;
    public static int y = 20;
    public static int back = 10;
    public static double eleTime=8.3;
    public static int angle= -90;
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
        //int pattern = 2;
        if (pattern == AutoCVUtil.Pattern.PATTERN_B) {
            dashboardTelemetry.addData("thing","1");
            dashboardTelemetry.update();
            Trajectory Pattern_A_Trajectory = drive.trajectoryBuilder()
                    //grabbing first skystone
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .back(back)
                    .strafeRight(50)
                    .forward(15)
                    .strafeLeft(30)
                   // .forward(100)


                    .lineTo(new Vector2d(85,-30),new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(100)))
                    //.lineTo(new Vector2d(-10,53),new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(70)))
                    //.forward(10)
                    //.lineTo(new Vector2d(-10,60),new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(65 )))


                    //.back(35)
                    .addMarker(4.0,() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.0);
                        manager.getHardware().getMotors().get("right_intake").setPower(0.0);
                        return Unit.INSTANCE;
                    })


                    .addMarker(4.0, () -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                        return Unit.INSTANCE;
                    })
                    .addMarker(5.0, () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.28);
                        return Unit.INSTANCE;
                    })


                    .addMarker(6.0, () -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(-0.20);
                        return Unit.INSTANCE;
                    })
                    .addMarker(8.0, () -> {
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
                        return Unit.INSTANCE;
                    })
                    .addMarker(9.0 , () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
                        return Unit.INSTANCE;
                    })

                    .addMarker(9.5, () -> {
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.28);
                        return Unit.INSTANCE;
                    })

                    .addMarker(9.5, () -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.25);
                        return Unit.INSTANCE;
                    })


                    .addMarker(10.5, () -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
                        return Unit.INSTANCE;
                    })


//                    .back(10)
//                    .addMarker(eleTime,() -> {
//                        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
//                        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
//                        return Unit.INSTANCE;
//                    })



                   // .back(5)
                    //.forward(35)


                    //finish dropping the skystone

                    //latch onto foundation




                    .build();

            drive.followTrajectorySync(Pattern_A_Trajectory);
          // drive.turnSync(Math.toRadians(500));
            Trajectory Pattern_A_Trajectory_Part2 = drive.trajectoryBuilder()
                    .back(30)

                    .strafeRight(45)
                    .addMarker(() -> {
                        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
                        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
                        return Unit.INSTANCE;
                    })
             /*       .addMarker(11.0, () -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.25);
                        return Unit.INSTANCE;
                    })
                    .addMarker(12.0, () -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
                        return Unit.INSTANCE;
                    })*/


                    .strafeRight(25)
                    .strafeLeft(50)
                    .forward(40)
                    //.lineTo(new Vector2d(69,y),new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(angle)))

                    //.forward(35)
                    .build();

          // drive.followTrajectorySync(Pattern_A_Trajectory_Part2);


        } else if (pattern == AutoCVUtil.Pattern.PATTERN_C || pattern == AutoCVUtil.Pattern.PATTERN_A)  {
            dashboardTelemetry.addData("thing","2");
            dashboardTelemetry.update();
            Trajectory Pattern_A_Trajectory = drive.trajectoryBuilder()
                    //grabbing first skystone
                    .addMarker(() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                        manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                        return Unit.INSTANCE;
                    })
                    .back(16)
                    .strafeRight(strafeLeft2)
                    .forward(17)
                    .strafeLeft(30)
                    // .forward(100)


                    .lineTo(new Vector2d(90,-30),new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(100)))
                    //.lineTo(new Vector2d(-10,53),new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(70)))
                    //.forward(10)
                    //.lineTo(new Vector2d(-10,60),new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(65 )))


                    //.back(35)
                    .addMarker(4.0,() -> {
                        manager.getHardware().getMotors().get("left_intake").setPower(0.0);
                        manager.getHardware().getMotors().get("right_intake").setPower(0.0);
                        return Unit.INSTANCE;
                    })


                    .addMarker(4.0, () -> {
                        manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                        return Unit.INSTANCE;
                    })
                    .addMarker(5.0, () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.28);
                        return Unit.INSTANCE;
                    })

/*
                    .addMarker(6.0, () -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(-0.20);
                        return Unit.INSTANCE;
                    })
                    .addMarker(8.0, () -> {
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
                        return Unit.INSTANCE;
                    })
                    .addMarker(9.0 , () -> {
                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
                        return Unit.INSTANCE;
                    })

                    .addMarker(9.5, () -> {
                        manager.getHardware().getServos().get("RotationServo").setPosition(0.28);
                        return Unit.INSTANCE;
                    })

                    .addMarker(9.5, () -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.25);
                        return Unit.INSTANCE;
                    })


                    .addMarker(10.5, () -> {
                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
                        return Unit.INSTANCE;
                    })


                    .back(10)
                    .addMarker(eleTime,() -> {
                        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
                        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
                        return Unit.INSTANCE;
                    })*/



                  /*  .back(5)
                    .forward(35)*/


                    //finish dropping the skystone

                    //latch onto foundation




                    .build();

            drive.followTrajectorySync(Pattern_A_Trajectory);
            //drive.turnSync(Math.toRadians(500));
            Trajectory Pattern_A_Trajectory_Part2 = drive.trajectoryBuilder()
                    .back(30)

                    .strafeRight(45)
                    .addMarker(() -> {
                        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
                        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
                        return Unit.INSTANCE;
                    })
                    /*       .addMarker(11.0, () -> {
                               manager.getHardware().getMotors().get("stoneElevator").setPower(0.25);
                               return Unit.INSTANCE;
                           })
                           .addMarker(12.0, () -> {
                               manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
                               return Unit.INSTANCE;
                           })*/


                    .strafeRight(25)
                    .strafeLeft(50)
                    .forward(40)
                    //.lineTo(new Vector2d(69,y),new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(angle)))

                    //.forward(35)
                    .build();

            //drive.followTrajectorySync(Pattern_A_Trajectory_Part2);

//        } else if (pattern == AutoCVUtil.Pattern.PATTERN_A || pattern==AutoCVUtil.Pattern.UNKNOWN) {
//            dashboardTelemetry.addData("thing","3");
//            dashboardTelemetry.update();
//
//            Trajectory Pattern_C_Trajectory = drive.trajectoryBuilder()
//                    //grabbing first skystone
//                    .back(22)
//                    .addMarker(() -> {
//                        manager.getHardware().getMotors().get("left_intake").setPower(0.5);
//                        manager.getHardware().getMotors().get("right_intake").setPower(-0.5);
//                        return Unit.INSTANCE;
//                    })
//                    .strafeRight(50)
//
//                    .forward(13)
//                    //end of grabbing the skystone
//
//                    //moving to foundation
//                    .strafeRight(20)
//
//                    .addMarker(4.0, () -> {
//                        manager.getHardware().getMotors().get("left_intake").setPower(0.0);
//                        manager.getHardware().getMotors().get("right_intake").setPower(0.0);
//                        return Unit.INSTANCE;
//                    })
//
//
//                    .addMarker(5.0, () -> {
//                        manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
//                        return Unit.INSTANCE;
//                    })
//                    .addMarker(6.0, () -> {
//                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.28);
//                        return Unit.INSTANCE;
//                    })
//
//
//                    //TODO check if this strafe is correct, could be 40
//                  //  .strafeRight(20)
//                   /* .addMarker(6.5, () -> {
//                        manager.getHardware().getMotors().get("stoneElevator").setPower(-0.15);
//                        return Unit.INSTANCE;
//                    })
//                    .addMarker(11.0, () -> {
//                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.25);
//                        return Unit.INSTANCE;
//                    })
//                    .addMarker(10.0, () -> {
//                        manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
//                        return Unit.INSTANCE;
//                    })
//                    .addMarker(11.0 , () -> {
//                        manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
//                        return Unit.INSTANCE;
//                    })
//
//                    .addMarker(11.5, () -> {
//                        manager.getHardware().getServos().get("RotationServo").setPosition(0.28);
//                        return Unit.INSTANCE;
//                    })
//
//
//
//                    .addMarker(11.0, () -> {
//                        manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
//                        return Unit.INSTANCE;
//                    })*/
//                    .lineTo(new Vector2d(-90, 28), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(-110)))
//                    //at foundation
//
//                    //drop the skystone
//
//                   // .back(5)
//
//                    //finish dropping the skystone
//
//                    //latch onto foundation
//                    /*.addMarker(() -> {
//                        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
//                        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
//                        return Unit.INSTANCE;
//                    })
//
//                    .back(10)
//                    .forward(35)*/
//                    .build();
//
//            drive.followTrajectorySync(Pattern_C_Trajectory);
//            //drive.turnSync(Math.toRadians(300));
//            Trajectory Pattern_C_Trajectory_Part2 = drive.trajectoryBuilder()
//
//                    .strafeRight(45)
//                    .addMarker(() -> {
//                        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
//                        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
//                        return Unit.INSTANCE;
//                    })
//
//
//                    .strafeRight(15)
//                    //.forward(35)
//                    .build();
//
//            //drive.followTrajectorySync(Pattern_C_Trajectory_Part2);
//

        }
        else {


        }


        while (!isStopRequested() && drive.isBusy()) {

            Pose2d currentPose = drive.getPoseEstimate();
           /*dashboardTelemetry.addData("backfirst",back);
           dashboardTelemetry.addData("strafe",strafeLeft);*/
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



