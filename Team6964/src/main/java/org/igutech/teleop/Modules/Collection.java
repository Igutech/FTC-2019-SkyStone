package org.igutech.teleop.Modules;

import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;

public class Collection extends Module {

    private GamepadService gamepadService;
    private double leftTrigger=0.0;
    private boolean leftbumper = false;
    public Collection(){super(700,"Collection");}

    public void init() {gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");}

    public void loop()
    {

        leftTrigger = gamepadService.getAnalog(1,"left_trigger");
        leftbumper = gamepadService.getDigital(1,"left_bumper");

        if(leftbumper)
        {
            Teleop.getInstance().getHardware().getMotors().get("left_intake").setPower(-0.25);
            Teleop.getInstance().getHardware().getMotors().get("right_intake").setPower(0.25);

        }
        else
        {

            Teleop.getInstance().getHardware().getMotors().get("left_intake").setPower(leftTrigger);
            Teleop.getInstance().getHardware().getMotors().get("right_intake").setPower((leftTrigger*-1));
        }

    }



}
