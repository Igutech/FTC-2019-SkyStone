package org.igutech.teleop.Modules;

import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;

public class Transfer extends Module {

    private GamepadService gamepadService;
    private boolean toggleleftBumper = false;
    private boolean togglerightBumper = false;
    private boolean previousButtonPositionleftBumper = false;
    private boolean previousButtonPositionrightBumper = false;

    public Transfer() {
        super(500, "Transfer");
    }

    public void init() {
        gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
    }

    public void loop() {

        boolean currentButtonPositionA = gamepadService.getDigital(1, "left_bumper");

        if (currentButtonPositionA && !previousButtonPositionleftBumper) {
            toggleleftBumper = !toggleleftBumper;
            if (toggleleftBumper) {
                Teleop.getInstance().getHardware().getServos().get("GrabberServo").setPosition(0.99);
            }
            if (!toggleleftBumper) {
                Teleop.getInstance().getHardware().getServos().get("GrabberServo").setPosition(0.65);
            }
        }
        previousButtonPositionleftBumper = currentButtonPositionA;

        boolean currentButtonPositionY = gamepadService.getDigital(1, "right_bumper");

        //resting 0.2, stack 0.88
        if (currentButtonPositionY && !previousButtonPositionrightBumper) {
            togglerightBumper = !togglerightBumper;
            if (togglerightBumper) {
                Teleop.getInstance().getHardware().getServos().get("RotationServo").setPosition(0.88);
            }
            if (!togglerightBumper) {
                Teleop.getInstance().getHardware().getServos().get("RotationServo").setPosition(0.2);
            }
        }
        previousButtonPositionrightBumper = currentButtonPositionY;

    }

}
