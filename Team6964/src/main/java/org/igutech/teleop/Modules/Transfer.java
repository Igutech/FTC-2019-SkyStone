package org.igutech.teleop.Modules;

import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;

public class Transfer extends Module {

    private GamepadService gamepadService;
    private boolean toggleX = false;
    private boolean previousButtonPositionX = false;
    private boolean currentButtonPositionX = false;

    public Transfer() {
        super(500, "FoundationServo");
    }

    public void init() {
        gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
    }

    public void loop() {

        currentButtonPositionX = gamepadService.getDigital(2, "x");

        if (currentButtonPositionX && !previousButtonPositionX) {
            toggleX = !toggleX;
            if (toggleX) {
                Teleop.getInstance().getHardware().getServos().get("TransferServo").setPosition(0.75);
            }
            if (!toggleX) {
                Teleop.getInstance().getHardware().getServos().get("TransferServo").setPosition(0.43);
            }
        }
        previousButtonPositionX = currentButtonPositionX;

    }

}
