package org.igutech.teleop.Modules;

import org.apache.commons.math3.util.FastMath;
import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;
import org.igutech.utils.control.PIDController;

import org.igutech.utils.FTCMath;

public class stoneElevator extends Module {

    private GamepadService gamepadService;
    double ManualPower = 0.0;
    double currentPos = 0;
    double startPos;

    boolean previousButtonPositionD_Up = false;
    boolean currentButtonPositionD_Up = false;

    boolean previousButtonPositionD_Down = false;
    boolean currentButtonPositionD_Down = false;

    boolean previousButtonPositionRightBumper = false;
    boolean currentButtonPositionRightBumper = false;


    boolean autoMode = false;
    boolean NeedtoEstimatePos = false;

    int level = 0;
    final int tickPerStone = 1440;


    PIDController elevatorController = new PIDController(0.15, 0, 0);

    public stoneElevator() {
        super(700, "stoneElevator");
    }

    public void init() {
        gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
    }

    public void start() {
        startPos = Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition();
    }

    public void loop() {
        currentPos = Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition() - startPos;
        currentButtonPositionD_Up = gamepadService.getDigital(2, "dpad_up");
        currentButtonPositionD_Down = gamepadService.getDigital(2, "dpad_down");
        currentButtonPositionRightBumper = gamepadService.getDigital(2, "right_bumper");

        ManualPower = gamepadService.getAnalog(2, "right_stick_y");
        double slowMoElevator = gamepadService.getAnalog(2, "right_trigger");
        double vdMultElevator = FTCMath.lerp(1, 0.4, FastMath.abs(slowMoElevator));
        ManualPower *= vdMultElevator;

        if (NeedtoEstimatePos) {
            double estimatedPos = (currentPos - tickPerStone) / tickPerStone;
            if (estimatedPos < 0)
                estimatedPos = 0;
            if (currentButtonPositionD_Up && !previousButtonPositionD_Up) {
                level = (int) Math.floor(estimatedPos) + 1;
                autoMode = true;
            } else if (currentButtonPositionD_Down && !previousButtonPositionD_Down) {
                level = (int) Math.floor(estimatedPos);
                autoMode = true;
            } else if (currentButtonPositionRightBumper && !previousButtonPositionRightBumper) {
                level = 1;
                autoMode = true;
            }
        } else {
            if (currentButtonPositionD_Up && !previousButtonPositionD_Up) {
                level++;
                autoMode = true;
            } else if (currentButtonPositionD_Down && !previousButtonPositionD_Down) {
                level--;
                autoMode = true;
            } else if (currentButtonPositionRightBumper && !previousButtonPositionRightBumper) {
                level = 1;
                autoMode = true;
            }
        }
        previousButtonPositionD_Down = currentButtonPositionD_Down;
        previousButtonPositionD_Up = currentButtonPositionD_Up;
        previousButtonPositionRightBumper = currentButtonPositionRightBumper;
        if (autoMode) {
            NeedtoEstimatePos = false;
        }

        if (Math.abs(ManualPower) > 0.01) {
            autoMode = false;
            NeedtoEstimatePos = true;
        }

        if (autoMode) {
            int setPoint = level * tickPerStone;
            elevatorController.updateSetpoint(setPoint);
            if (Math.abs(setPoint - Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition()) > 100) {
                elevatorController.reset(Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition());
            }
            double power = elevatorController.update(Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition());
            power = FTCMath.clamp(-0.5, 0.5, power);
            Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(power);
        } else {
            Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(ManualPower);
        }

    }

}
