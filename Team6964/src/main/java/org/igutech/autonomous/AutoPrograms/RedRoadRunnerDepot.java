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

import kotlin.Unit;

import static org.igutech.autonomous.roadrunner.MecanumDriveBase.BASE_CONSTRAINTS;

@Config
@Autonomous(name = "RedRoadRunnerDepot", group = "igutech")
public class RedRoadRunnerDepot extends LinearOpMode {

    public static int strafe = 25;
    public static int strafe2 = 48;

    public static int turn=90;

    public static int forward = 10;

    public static int x = 120;
    public static int y = -38;
    DriveConstraints slowConstraints = new DriveConstraints(
            35, 20, BASE_CONSTRAINTS.maxJerk,
            BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk);

   FtcDashboard dashboard = FtcDashboard.getInstance();
   Telemetry dashboardTelemetry = dashboard.getTelemetry();

   @Override
   public void runOpMode() throws InterruptedException {

      AutoUtilManager manager = new AutoUtilManager(hardwareMap, "TestRoadrunner");
      manager.getDriveUtil().resetEncoders();
      IguMecanumDriveBase drive = new IguMecanumDriveBase(manager);
      drive.setPoseEstimate(new Pose2d(50.0,-63.0,Math.toRadians(-180.0)));

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
      if (pattern == AutoCVUtil.Pattern.PATTERN_A) {


      } else if (pattern == AutoCVUtil.Pattern.PATTERN_B) {
          Trajectory patternBMoveToFoundation=drive.trajectoryBuilder()
                  .back(10.0)

                  .strafeRight(strafe2)

                  .build();
          drive.followTrajectorySync(patternBMoveToFoundation);

          Trajectory two = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                  .addMarker(() -> {
                      manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                      manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                      return Unit.INSTANCE;
                  })
                  .forward(forward)
                  .strafeLeft(strafe)
                  .addMarker(() -> {
                      manager.getHardware().getMotors().get("left_intake").setPower(0);
                      manager.getHardware().getMotors().get("right_intake").setPower(0);
                      return Unit.INSTANCE;
                  })

          .build();
          drive.followTrajectorySync(two);

          Trajectory three=drive.trajectoryBuilder()
                  .setReversed(false)
                  //now backing up toward the foundation x is how far back and y is how close to the foundation
                  .addMarker(() -> {
                      manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                      return Unit.INSTANCE;
                  })
                  .lineTo(new Vector2d(x,y),new LinearInterpolator(Math.toRadians(180.0), Math.toRadians(0.0)))
                  .addMarker(() -> {
                      manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
                      return Unit.INSTANCE;
                  })
//                  //turn so the foundation grabbers can latch
                  .lineTo(new Vector2d(125,-38.0),new LinearInterpolator(Math.toRadians(180.0),Math.toRadians(90.0)))
//                  //back up a bit more for safety
                  .lineTo(new Vector2d(125,-30.0),new LinearInterpolator(Math.toRadians(-90.0),Math.toRadians(0.0)))
                  .build();
          drive.followTrajectorySync(three);


          manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
          manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
          sleep(500);

          Trajectory patternMoveFoundation = drive.trajectoryBuilder()
                  .forward(45)
                  .build();
          drive.followTrajectorySync(patternMoveFoundation);
          drive.turnSync(Math.toRadians(-90));

          manager.getHardware().getMotors().get("stoneElevator").setPower(0.6);
          sleep(600);
          manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
          sleep(400);
          manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
          sleep(400);

          manager.getHardware().getMotors().get("stoneElevator").setPower(-0.6);
          sleep(600);
          manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
          manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
          manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
          sleep(200);

          Trajectory patternBReleaseStone = drive.trajectoryBuilder()
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
                  .forward(50)

                  .build();
          drive.followTrajectorySync(patternBReleaseStone);


//          Trajectory patternBReleaseStone = drive.trajectoryBuilder()
//                  .forward(1)
//                  .build();
//          drive.followTrajectorySync(patternBReleaseStone);
//          manager.getHardware().getMotors().get("stoneElevator").setPower(0.6);
//          sleep(600);
//          manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
//          sleep(400);
//          manager.getHardware().getServos().get("RotationServo").setPosition(0.28);
//          sleep(600);
//
//          manager.getHardware().getMotors().get("stoneElevator").setPower(-0.6);
//          sleep(600);
//          manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
//
//
//          Trajectory patternBPark = drive.trajectoryBuilder()
//                  .back(15)
//                  .strafeRight(20)
//                  .forward(50)
//                  .build();
//          drive.followTrajectorySync(patternBPark);



//
//          drive.setPoseEstimate(new Pose2d(50.0,-63.0,Math.toRadians(-180.0)));
//
//          Trajectory secondStone = drive.trajectoryBuilder()
//                  .strafeRight(25.0)
//                  .forward(95.0)
//                  .strafeRight(15.0)
//                  .forward(7.0)
//                  .strafeLeft(13.0)
//                  .lineTo(new Vector2d(55.0,-38.0),new LinearInterpolator(Math.toRadians(180.0), Math.toRadians(0.0)))
//                  .build();
//
//          drive.followTrajectorySync(secondStone);


      } else if (pattern == AutoCVUtil.Pattern.PATTERN_C ) {

          Trajectory patternBMoveToFoundation=drive.trajectoryBuilder()
                  .back(4)

                  .strafeRight(strafe2)

                  .build();
          drive.followTrajectorySync(patternBMoveToFoundation);

          Trajectory patternCIntake = new TrajectoryBuilder(drive.getPoseEstimate(), slowConstraints)
                  .addMarker(() -> {
                      manager.getHardware().getMotors().get("left_intake").setPower(0.75);
                      manager.getHardware().getMotors().get("right_intake").setPower(-0.75);
                      return Unit.INSTANCE;
                  })
                  .forward(7.5)
                  .strafeLeft(30)
                  .back(3)
                  .addMarker(() -> {
                      manager.getHardware().getMotors().get("left_intake").setPower(0.0);
                      manager.getHardware().getMotors().get("right_intake").setPower(0.0);
                      return Unit.INSTANCE;
                  })
                  .build();
          drive.followTrajectorySync(patternCIntake);


          Trajectory three=drive.trajectoryBuilder()
                  .setReversed(false)
                  //now backing up toward the foundation x is how far back and y is how close to the foundation
                  .addMarker(() -> {
                      manager.getHardware().getServos().get("TransferServo").setPosition(0.75);
                      return Unit.INSTANCE;
                  })
                  .lineTo(new Vector2d(x,y),new LinearInterpolator(Math.toRadians(180.0), Math.toRadians(0.0)))
                  .addMarker(() -> {
                      manager.getHardware().getServos().get("GrabberServo").setPosition(0.3);
                      return Unit.INSTANCE;
                  })
//                  //turn so the foundation grabbers can latch
                  .lineTo(new Vector2d(125,-38.0),new LinearInterpolator(Math.toRadians(180.0),Math.toRadians(90.0)))
//                  //back up a bit more for safety
                  .lineTo(new Vector2d(125,-33.0),new LinearInterpolator(Math.toRadians(-90.0),Math.toRadians(0.0)))
                  .build();
          drive.followTrajectorySync(three);


          manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.6);
          manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.9);
          sleep(500);

          Trajectory patternMoveFoundation = drive.trajectoryBuilder()
                  .forward(50)
                  .build();
          drive.followTrajectorySync(patternMoveFoundation);
          drive.turnSync(Math.toRadians(-90));

          manager.getHardware().getMotors().get("stoneElevator").setPower(0.6);
          sleep(600);
          manager.getHardware().getMotors().get("stoneElevator").setPower(0.0);
          sleep(400);
          manager.getHardware().getServos().get("RotationServo").setPosition(0.95);
          sleep(400);

          manager.getHardware().getMotors().get("stoneElevator").setPower(-0.6);
          sleep(600);
          manager.getHardware().getServos().get("GrabberServo").setPosition(0.1);
          manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.1);
          manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.2);
          sleep(200);

          Trajectory patternBReleaseStone = drive.trajectoryBuilder()
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
                  .forward(50)

                  .build();
          drive.followTrajectorySync(patternBReleaseStone);

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



