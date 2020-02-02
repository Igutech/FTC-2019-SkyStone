package org.igutech.teleop.Modules;

import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;

public class EncoderTest extends Module {
    public EncoderTest( ) {
        super(1200, "EncoderTest");
    }
    public void loop()
    {
        Teleop.getInstance().telemetry.addData("Current tick",
                Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition());
    }
}
