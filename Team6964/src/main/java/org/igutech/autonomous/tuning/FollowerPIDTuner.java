package org.igutech.autonomous.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.util.AutoUtilManager;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive")
public class FollowerPIDTuner extends LinearOpMode {
    public static double DISTANCE = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "TestRoadrunner");
        IguMecanumDriveBase drive = new IguMecanumDriveBase(manager);


        drive.setPoseEstimate(new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(drive.trajectoryBuilder().forward(DISTANCE).build());

        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
            System.out.println(String.format("\t%.5f\t%.5f", drive.getFollower().elapsedTime(), drive.getLastError().getX()));
        }
        System.out.println("DONE");
    }
}
