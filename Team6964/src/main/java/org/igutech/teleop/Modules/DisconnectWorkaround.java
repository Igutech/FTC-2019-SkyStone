package org.igutech.teleop.Modules;

import org.igutech.teleop.Service;
import org.igutech.teleop.Teleop;

public class DisconnectWorkaround extends Service {

    public DisconnectWorkaround() {
        super("DisconnectWorkaround");
    }

    @Override
    public void initLoop() {
        Teleop.getInstance().telemetry.addData("status", "loop test... waiting for start");
    }
}
