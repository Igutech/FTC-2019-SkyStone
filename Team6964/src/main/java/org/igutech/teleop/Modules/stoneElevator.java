package org.igutech.teleop.Modules;

import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;
import org.igutech.utils.control.PIDController;

import static org.igutech.utils.FTCMath.clamp;

public class stoneElevator extends Module {

    private GamepadService gamepadService;
    boolean reset = false;
    double currentPos = 0;
    double startPos;

    boolean previousButtonPositionD_Up = false;
    boolean toggleD_Up = false;
    boolean currentButtonPositionD_Up = false;

    boolean previousButtonPositionD_Down = false;
    boolean toggleD_Down = false;
    boolean currentButtonPositionD_Down = false;


    boolean autoMode = false;
    boolean NeedtoEstimatePos = false;

    int level = 0;
    final int tickPerStone = 1440;


    PIDController elevatorController = new PIDController(0.5, 0, 0);

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
            }

        } else {
            if (currentButtonPositionD_Up && !previousButtonPositionD_Up) {
                level++;
                autoMode = true;
            } else if (currentButtonPositionD_Down && !previousButtonPositionD_Down) {
                level--;
                autoMode = true;
            }
        }
        previousButtonPositionD_Down = currentButtonPositionD_Down;
        previousButtonPositionD_Up = currentButtonPositionD_Up;
        if (autoMode) {
            NeedtoEstimatePos = false;
        }

        if (Math.abs(gamepadService.getAnalog(2, "right_stick_y")) > 0.01) {
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
            power = clamp(-0.5, 0.5, power);
            Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(power);
        } else {
            Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(gamepadService.getAnalog(2, "right_stick_y"));
        }

    }

}
