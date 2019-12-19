package org.igutech.teleop.Modules;

import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;
import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.util.AutoUtilManager;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.util.AutoUtilManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RROdometry extends Module {

    private GamepadService gamepadService;
    private boolean toggleX = false;
    private boolean previousButtonPositionX = false;
    private boolean currentButtonPositionX = false;

    public RROdometry(){super(200,"RROdometry");}
    public void init() {gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");}

    public void loop() {
        IguMecanumDriveBase drive = new IguMecanumDriveBase(null);
    }

}
