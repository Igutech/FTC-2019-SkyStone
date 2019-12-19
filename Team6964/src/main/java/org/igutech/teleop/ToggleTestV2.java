
package org.igutech.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.igutech.teleop.Modules.GamepadService;

@TeleOp
public class ToggleTestV2 extends LinearOpMode {

    private GamepadService gamepadService;
    private boolean toggle = false;
    private boolean previousButtonPosition = false;
    private boolean currentButtonPosition = false;

    @Override
    public void runOpMode()
    {
        gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");

        telemetry.addData("ready","to go");
        telemetry.update();
        waitForStart();
        while(!isStopRequested()& opModeIsActive()) {
            currentButtonPosition = gamepadService.getDigital(1, "x");

            if (currentButtonPosition && !previousButtonPosition) {
                toggle = !toggle;
                if(toggle){Teleop.getInstance().getHardware().getServos().get("RotationServo").setPosition(0.9);}
                if(!toggle){Teleop.getInstance().getHardware().getServos().get("RotationServo").setPosition(0.1);}
            }
            previousButtonPosition = currentButtonPosition;
        }

    }
}