package org.igutech.teleop.Modules;


import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;

public class Capping extends Module {

    private GamepadService gamepadService;
    private boolean toggleleftDapd = false;
    private boolean previousButtonPositionleftDpad = false;
    private boolean currentButtonPositionleftPad = false;


    public Capping(){super(100,"Capping");}


    public void init() {gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
    }

    public void loop()
    {

        currentButtonPositionleftPad = gamepadService.getDigital(1, "dpad_left");

        if (currentButtonPositionleftPad && !previousButtonPositionleftDpad) {
            toggleleftDapd = !toggleleftDapd;
            if(toggleleftDapd){Teleop.getInstance().getHardware().getServos().get("CapServo").setPosition(0.62);}
            if(!toggleleftDapd){Teleop.getInstance().getHardware().getServos().get("CapServo").setPosition(0.29);}
        }
        previousButtonPositionleftDpad = currentButtonPositionleftPad;

    }

}
