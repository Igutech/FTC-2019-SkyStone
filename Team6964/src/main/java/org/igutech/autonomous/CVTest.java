package org.igutech.autonomous;/*
package org.igutech.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.igutech.autonomous.util.AutoUtilManager;
@Disabled
@Autonomous(name="CVTest", group="igutech")
public class CVTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "CVTest");

        manager.getCvUtil().activate();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while(opModeIsActive()) {

            telemetry.addData("Detection", manager.getCvUtil().getPattern());
            telemetry.update();
            Thread.sleep(250);
        }
        manager.getCvUtil().shutdown();

    }

}

*/
