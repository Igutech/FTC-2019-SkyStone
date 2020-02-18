package org.igutech.autonomous.AutoPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.util.AutoUtilManager;

@Autonomous
public class BlueRoadRunnerSite extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "BlueRoadRunnerSite");
        manager.getDriveUtil().resetEncoders();
        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.55);
        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.99);
        manager.getHardware().getServos().get("RotationServo").setPosition(0.2);
        manager.getHardware().getServos().get("CapServo").setPosition(0.29);
        manager.getHardware().getServos().get("GrabberServo").setPosition(0.65);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        IguMecanumDriveBase drive = new IguMecanumDriveBase(manager);
        Trajectory leftTrajectory = drive.trajectoryBuilder()
                .forward(10)
                .build();
        while (!opModeIsActive() && !isStopRequested()) {

            telemetry.addData("status", "waiting for start command...");

            telemetry.update();

        }
        if (isStopRequested()) return;
        //drive.followTrajectorySync(leftTrajectory);

        manager.getHardware().getServos().get("FoundationServo_left").setPosition(0.93);
        manager.getHardware().getServos().get("FoundationServo_right").setPosition(0.6);




    }

}
