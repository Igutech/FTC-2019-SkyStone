package org.igutech.teleop.Modules;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.FastMath;
import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;
import org.igutech.utils.control.PIDController;

import org.igutech.utils.FTCMath;

@Config
public class StoneElevator extends Module {

    private GamepadService gamepadService;


    public StoneElevator() {
        super(710, "StoneElevator");
    }

    public void init() {
        gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
    }

    public void start() {

    }

    public void loop() {

        double down = gamepadService.getAnalog(1,"left_trigger");
        double up = gamepadService.getAnalog(1,"right_trigger");

        if(down>0.0){
            Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(down*-1);
        } else if(up>0.0){
            Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(up);
        }
    }


}