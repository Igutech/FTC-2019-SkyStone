package org.igutech.teleop.Modules;

import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;

public class Transfer extends Module {

    private GamepadService gamepadService;
    private boolean toggleA = false;
    private boolean toggleY = false;
    private boolean previousButtonPositionA = false;
    private boolean previousButtonPositionY = false;

    public Transfer() {
        super(500, "Transfer");
    }

    public void init() {
        gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
    }

    public void loop() {

        boolean currentButtonPositionA = gamepadService.getDigital(2, "a");

        if (currentButtonPositionA && !previousButtonPositionA) {
            toggleA = !toggleA;
            if (toggleA) {
                Teleop.getInstance().getHardware().getServos().get("GrabberServo").setPosition(0.95);
            }
            if (!toggleA) {
                Teleop.getInstance().getHardware().getServos().get("GrabberServo").setPosition(0.65);
            }
        }
        previousButtonPositionA = currentButtonPositionA;

        boolean currentButtonPositionY = gamepadService.getDigital(2, "y");

        //resting 0.2, stack 0.88
        if (currentButtonPositionY && !previousButtonPositionY) {
            toggleY = !toggleY;
            if (toggleY) {
                Teleop.getInstance().getHardware().getServos().get("RotationServo").setPosition(0.88);
            }
            if (!toggleY) {
                Teleop.getInstance().getHardware().getServos().get("RotationServo").setPosition(0.2);
            }
        }
        previousButtonPositionY = currentButtonPositionY;

    }

}
