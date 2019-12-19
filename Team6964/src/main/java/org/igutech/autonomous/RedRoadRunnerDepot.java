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
@Autonomous(name = "RedRoadRunnerDepot", group = "igutech")
public class RedRoadRunnerDepot extends LinearOpMode {
   public static int strafeLeft2 = 20;
  // public static int x = -80;
   //public static int y = -75;
   //public static int back=13;
   public static int back2=30;
   //public static int pattern = 2;


   //public static double eleTime=8.7;
   //public static int angle= 135;
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
      if (pattern == AutoCVUtil.Pattern.PATTERN_C) {

         Trajectory Pattern_A_Trajectory = drive.trajectoryBuilder()
                 .addMarker(() -> {
                    manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                    manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                    return Unit.INSTANCE;
                 })
                 .strafeRight(5)
                 .back(30)
                 //-16,-50 -65 degree
                 //best option (-12,-55) -75 angle
                 .lineTo(new Vector2d(-12,-50),new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(-75)))
                 .back(back2)
                 .addMarker(5.0,() -> {
                    manager.getHardware().getMotors().get("left_intake").setPower(0.0);
                    manager.getHardware().getMotors().get("right_intake").setPower(0.0);
                    return Unit.INSTANCE;
                 })


                 .addMarker(5.0, () -> {
                    manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                    return Unit.INSTANCE;
                 })
                 .addMarker(6.0, () -> {
                    manager.getHardware().getServos().get("GrabberServo").setPosition(0.28);
                    return Unit.INSTANCE;
                 })


                 .addMarker(7.0, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(-0.20);
                    return Unit.INSTANCE;
                 })
                 .addMarker(9.0, () -> {
                    manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
                    return Unit.INSTANCE;
                 })
                 .addMarker(10.5 , () -> {
                    manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
                    return Unit.INSTANCE;
                 })

                 .addMarker(11.0, () -> {
                    manager.getHardware().getServos().get("RotationServo").setPosition(0.28);
                    return Unit.INSTANCE;
                 })
                 .addMarker(11.0, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.25);
                    return Unit.INSTANCE;
                 })

                 .lineTo(new Vector2d(-100, -70),new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(135)))
                 //latch onto foundation
                 .addMarker(11.5, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.25);
                    return Unit.INSTANCE;
                 })


                 .addMarker(12.5, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
                    return Unit.INSTANCE;
                 })
                 .back(10)

                 .addMarker(9.0,() -> {
                    manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
                    manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
                    return Unit.INSTANCE;
                 })


                // .back(5)

                 .forward(65)


                 .build();

         drive.followTrajectorySync(Pattern_A_Trajectory);

         drive.turnSync(Math.toRadians(-100));
         Trajectory Pattern_C_Trajectory_Part2 = drive.trajectoryBuilder()

                 .strafeLeft(45)
                 .addMarker(() -> {
                    manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
                    manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
                    return Unit.INSTANCE;
                 })
                 .addMarker(11.0, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.25);
                    return Unit.INSTANCE;
                 })
                 .addMarker(12.0, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
                    return Unit.INSTANCE;
                 })
                 .strafeLeft(15)
                 .strafeRight(55)
                 .forward(40)
                 .build();

         drive.followTrajectorySync(Pattern_C_Trajectory_Part2);



      } else if (pattern == AutoCVUtil.Pattern.PATTERN_B) {
         Trajectory Pattern_B_Trajectory = drive.trajectoryBuilder()

                 .addMarker(() -> {
                    manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                    manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                    return Unit.INSTANCE;
                 })
                 .addMarker(5.0, () -> {
                    manager.getHardware().getMotors().get("left_intake").setPower(0.0);
                    manager.getHardware().getMotors().get("right_intake").setPower(0.0);
                    return Unit.INSTANCE;
                 })
                 .back(13)
                 .strafeRight(53)
                 .forward(10)
                 .strafeLeft(20)
                 .addMarker(5.0, () -> {
                    manager.getHardware().getMotors().get("left_intake").setPower(0.0);
                    manager.getHardware().getMotors().get("right_intake").setPower(0.0);
                    return Unit.INSTANCE;
                 })


                 .addMarker(5.0, () -> {
                    manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                    return Unit.INSTANCE;
                 })
                 .addMarker(6.0, () -> {
                    manager.getHardware().getServos().get("GrabberServo").setPosition(0.28);
                    return Unit.INSTANCE;
                 })


                 //TODO check if this strafe is correct, could be 40
                 .strafeLeft(20)
                 .addMarker(6.5, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(-0.15);
                    return Unit.INSTANCE;
                 })
                 .addMarker(11.0, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.25);
                    return Unit.INSTANCE;
                 })
                 .addMarker(10.0, () -> {
                    manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
                    return Unit.INSTANCE;
                 })
                 .addMarker(11.0 , () -> {
                    manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
                    return Unit.INSTANCE;
                 })

                 .addMarker(11.5, () -> {
                    manager.getHardware().getServos().get("RotationServo").setPosition(0.28);
                    return Unit.INSTANCE;
                 })



                 .addMarker(11.5, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
                    return Unit.INSTANCE;
                 })
                 .lineTo(new Vector2d(-80,-30), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(105)))

                 //finish dropping the skystone

                 //latch onto foundation
                 .addMarker(11.5, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.25);
                    return Unit.INSTANCE;
                 })


                 .addMarker(12.5, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
                    return Unit.INSTANCE;
                 })
                 .back(10)

                 .addMarker(8.7,() -> {
                     manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
                     manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
                     return Unit.INSTANCE;
                 })


                 //.back(8)

                 .forward(65)



                 .build();

         drive.followTrajectorySync(Pattern_B_Trajectory);
         drive.turnSync(Math.toRadians(-100));
         Trajectory Pattern_B_Trajectory_Part2 = drive.trajectoryBuilder()

                 .strafeLeft(25)
                 .addMarker(() -> {
                    manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
                    manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
                    return Unit.INSTANCE;
                 })
                 .addMarker(11.0, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.25);
                    return Unit.INSTANCE;
                 })
                 .addMarker(11.5, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
                    return Unit.INSTANCE;
                 })


                 .strafeLeft(10)
                 .strafeRight(50)
                 .forward(35)
                 .build();

         drive.followTrajectorySync(Pattern_B_Trajectory_Part2);

      } else if (pattern == AutoCVUtil.Pattern.PATTERN_A ) {

         Trajectory Pattern_C_Trajectory = drive.trajectoryBuilder()

                 .addMarker(() -> {
                    manager.getHardware().getMotors().get("left_intake").setPower(0.5);
                    manager.getHardware().getMotors().get("right_intake").setPower(-0.5);
                    return Unit.INSTANCE;
                 })
                 .addMarker(4.0, () -> {
                    manager.getHardware().getMotors().get("left_intake").setPower(0.0);
                    manager.getHardware().getMotors().get("right_intake").setPower(0.0);
                    return Unit.INSTANCE;
                 })
                 .back(3)
                 .strafeRight(50)
                 .forward(10)
                 .strafeLeft(20)
                 .addMarker(4.0, () -> {
                    manager.getHardware().getMotors().get("left_intake").setPower(0.0);
                    manager.getHardware().getMotors().get("right_intake").setPower(0.0);
                    return Unit.INSTANCE;
                 })


                 .addMarker(5.0, () -> {
                    manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                    return Unit.INSTANCE;
                 })
                 .addMarker(6.0, () -> {
                    manager.getHardware().getServos().get("GrabberServo").setPosition(0.28);
                    return Unit.INSTANCE;
                 })


                 //TODO check if this strafe is correct, could be 40
                 .strafeLeft(25)
                 .addMarker(6.5, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(-0.15);
                    return Unit.INSTANCE;
                 })
                 .addMarker(11.0, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.25);
                    return Unit.INSTANCE;
                 })
                 .addMarker(10.0, () -> {
                    manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
                    return Unit.INSTANCE;
                 })
                 .addMarker(11.0 , () -> {
                    manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
                    return Unit.INSTANCE;
                 })

                 .addMarker(11.5, () -> {
                    manager.getHardware().getServos().get("RotationServo").setPosition(0.28);
                    return Unit.INSTANCE;
                 })



                 .addMarker(11.5, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
                    return Unit.INSTANCE;
                 })
                 .lineTo(new Vector2d(-85,-30), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(105)))
                 .back(10)

                 //finish dropping the skystone

                 //latch onto foundation
                 .addMarker(11.5, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.25);
                    return Unit.INSTANCE;
                 })
                 .addMarker(9.0,() -> {
                    manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
                    manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
                    return Unit.INSTANCE;
                 })

                 .addMarker(12.5, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
                    return Unit.INSTANCE;
                 })


                // .back(8)

                 .forward(65)



                 .build();

         drive.followTrajectorySync(Pattern_C_Trajectory);
         drive.turnSync(Math.toRadians(-100));
         Trajectory Pattern_C_Trajectory_Part2 = drive.trajectoryBuilder()

                 .strafeLeft(25)
                 .addMarker(() -> {
                    manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
                    manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
                    return Unit.INSTANCE;
                 })
                 .addMarker(11.0, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.25);
                    return Unit.INSTANCE;
                 })
                 .addMarker(12.0, () -> {
                    manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
                    return Unit.INSTANCE;
                 })


                 .strafeLeft(15)
                 .strafeRight(52)
                 .forward(35)
                 .build();

         drive.followTrajectorySync(Pattern_C_Trajectory_Part2);

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



