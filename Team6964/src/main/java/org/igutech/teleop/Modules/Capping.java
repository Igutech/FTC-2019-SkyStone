package org.igutech.teleop.Modules;


import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;

public class Capping extends Module {

    private GamepadService gamepadService;
    private boolean toggleX = false;
    private boolean previousButtonPositionX = false;
    private boolean currentButtonPositionX = false;

    public Capping(){super(100,"Capping");}


    public void init() {gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
    }

    public void loop()
    {

        currentButtonPositionX = gamepadService.getDigital(1, "x");

        if (currentButtonPositionX && !previousButtonPositionX) {
            toggleX = !toggleX;
            if(toggleX){Teleop.getInstance().getHardware().getServos().get("CapServo").setPosition(0.77);}
            if(!toggleX){Teleop.getInstance().getHardware().getServos().get("CapServo").setPosition(0.53);}
        }
        previousButtonPositionX = currentButtonPositionX;

    }

}
