package org.igutech.teleop.Modules;

import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;

public class FoundationServo extends Module {

    private GamepadService gamepadService;
    private boolean toggle = false;
    private boolean previousButtonPosition = false;
    private boolean currentButtonPosition = false;

    public FoundationServo() {
        super(300, "FoundationServo");
    }

    public void init() {
        gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
    }

    public void loop() {

        currentButtonPosition = gamepadService.getDigital(1, "right_bumper");

        //right; free 0.99, holding 0.6
        if (currentButtonPosition && !previousButtonPosition) {
            toggle = !toggle;
            if (toggle) {
                Teleop.getInstance().getHardware().getServos().get("FoundationServo_left").setPosition(0.93);
                Teleop.getInstance().getHardware().getServos().get("FoundationServo_right").setPosition(0.6);
            }
            if (!toggle) {
                Teleop.getInstance().getHardware().getServos().get("FoundationServo_left").setPosition(0.55);
                Teleop.getInstance().getHardware().getServos().get("FoundationServo_right").setPosition(0.99);
            }
        }
        previousButtonPosition = currentButtonPosition;

    }

}
