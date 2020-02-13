package org.igutech.teleop.Modules;

import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;

public class Collection extends Module {

    private GamepadService gamepadService;
    private double leftTrigger = 0.0;
    private boolean leftbumper = false;

    public Collection() {
        super(800, "Collection");
    }

    public void init() {
        gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
    }

    public void loop() {

        leftTrigger = gamepadService.getAnalog(1, "left_trigger");
        leftbumper = gamepadService.getDigital(1, "left_bumper");

        if (leftbumper) {
            Teleop.getInstance().getHardware().getMotors().get("left_intake").setPower(0.6);
            Teleop.getInstance().getHardware().getMotors().get("right_intake").setPower(-0.6);
            Teleop.getInstance().getHardware().getMotors().get("transferMotor").setPower(1.0);

        }

        if (leftTrigger > 0.01) {
            Teleop.getInstance().getHardware().getMotors().get("left_intake").setPower(-0.6);
            Teleop.getInstance().getHardware().getMotors().get("right_intake").setPower(0.6);
            Teleop.getInstance().getHardware().getMotors().get("transferMotor").setPower(-1.0);
        }
        if (leftTrigger < 0.01 && !leftbumper) {
            Teleop.getInstance().getHardware().getMotors().get("left_intake").setPower(0.0);
            Teleop.getInstance().getHardware().getMotors().get("right_intake").setPower(0.0);
            Teleop.getInstance().getHardware().getMotors().get("transferMotor").setPower(0.0);
        }
    }


}
