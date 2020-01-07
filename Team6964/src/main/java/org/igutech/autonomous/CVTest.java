package org.igutech.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.igutech.autonomous.util.AutoUtilManager;
@Disabled
@Autonomous(name="CVTest", group="igutech")
public class CVTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "CVTest");

        manager.getCvUtil().activate();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        while (!opModeIsActive() && !isStopRequested()) {
            dashboardTelemetry.addData("status", "waiting for start command...");
            dashboardTelemetry.update();
        }

        while(opModeIsActive()) {

            dashboardTelemetry.addData("Detection", manager.getCvUtil().getPattern());
            dashboardTelemetry.update();
        }
        manager.getCvUtil().shutdown();

    }

}

