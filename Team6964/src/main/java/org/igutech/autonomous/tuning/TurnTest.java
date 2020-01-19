package org.igutech.autonomous.tuning;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.util.AutoUtilManager;
@Config
@Disabled
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 180; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "TurnTest");
        manager.getDriveUtil().resetEncoders();
        IguMecanumDriveBase drive = new IguMecanumDriveBase(manager);
        telemetry.addData("ready","to go");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        drive.turnSync(Math.toRadians(ANGLE));
    }
}