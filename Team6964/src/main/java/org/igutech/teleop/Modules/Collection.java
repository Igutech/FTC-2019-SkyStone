package org.igutech.teleop.Modules;

import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;

public class Collection extends Module {

    private GamepadService gamepadService;
    private boolean toggleY = false;
    private boolean previousButtonPositionY = false;
    private boolean currentButtonPositionY = false;

    public Collection() {
        super(800, "Collection");
    }

    public void init() {
        gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
    }

    public void loop() {

       currentButtonPositionY=gamepadService.getDigital(1,"y");
        if (currentButtonPositionY && !previousButtonPositionY) {
            toggleY = !toggleY;
            if(toggleY){
                Teleop.getInstance().getHardware().getMotors().get("left_intake").setPower(-0.6);
                Teleop.getInstance().getHardware().getMotors().get("right_intake").setPower(0.6);
                Teleop.getInstance().getHardware().getMotors().get("transferMotor").setPower(-1.0);
            }
            if(!toggleY){
                Teleop.getInstance().getHardware().getMotors().get("left_intake").setPower(0.0);
                Teleop.getInstance().getHardware().getMotors().get("right_intake").setPower(0.0);
                Teleop.getInstance().getHardware().getMotors().get("transferMotor").setPower(0.0);
            }
        }
       previousButtonPositionY = currentButtonPositionY;

    }


}
