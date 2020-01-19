package org.igutech.teleop.Modules;

import com.acmerobotics.dashboard.config.Config;

import org.apache.commons.math3.util.FastMath;
import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;
import org.igutech.utils.control.PIDController;

import org.igutech.utils.FTCMath;

@Config
public class StoneElevator extends Module {

    private GamepadService gamepadService;
    private double manualPower = 0.0;
    private double startPos;

    private boolean previousButtonPositionDpadUp = false;
    private boolean currentButtonPositionDpadUp = false;

    private boolean previousButtonPositionDpadDown = false;
    private boolean currentButtonPositionDpadDown = false;

    private boolean previousButtonPositionRightBumper = false;
    private boolean currentButtonPositionRightBumper = false;


    private boolean autoMode = false;
    private boolean needToEstimatePos = false;

    private int level = 0;
    private final int TICK_PER_STONE = 120;

    public static double p = 0.02;
    public static double i = 0.00;
    public static double d = 0.00;

    private PIDController elevatorController = new PIDController(p, i, d);

    public StoneElevator() {
        super(700, "StoneElevator");
    }

    public void init() {
        gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
    }

    public void start() {
        startPos = Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition();
    }

    public void loop() {
//        Teleop.getInstance().telemetry.addData("start tick",startPos);
//        Teleop.getInstance().telemetry.addData("Current tick",
//                Teleop.getInstance().getHardware().getMotors ().get("stoneElevator").getCurrentPosition());
//        Teleop.getInstance().telemetry.addData("power",gamepadService.getAnalog(2, "right_stick_y"));
        currentButtonPositionDpadUp = gamepadService.getDigital(2, "dpad_up");
        currentButtonPositionDpadDown = gamepadService.getDigital(2, "dpad_down");
        currentButtonPositionRightBumper = gamepadService.getDigital(2, "right_bumper");

        manualPower = -1 * gamepadService.getAnalog(2, "right_stick_y");
        double slowMoElevator = FTCMath.lerp(1, 0.4, FastMath.abs(gamepadService.getAnalog(2, "right_trigger")));
        manualPower *= slowMoElevator;
        Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(manualPower);

//        if (needToEstimatePos) {
//            double currentPos = Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition() - startPos;
//            double estimatedPos = (currentPos - TICK_PER_STONE) / TICK_PER_STONE;
//            if (estimatedPos < 0)
//                estimatedPos = 0;
//            if (currentButtonPositionDpadUp && !previousButtonPositionDpadUp) {
//                level = (int) Math.floor(estimatedPos) + 1;
//                autoMode = true;
//            } else if (currentButtonPositionDpadDown && !previousButtonPositionDpadDown) {
//                level = (int) Math.floor(estimatedPos);
//                autoMode = true;
//            } else if (currentButtonPositionRightBumper && !previousButtonPositionRightBumper) {
//                level = 1;
//                autoMode = true;
//            }
//        } else {
//            if (currentButtonPositionDpadUp && !previousButtonPositionDpadUp) {
//                level++;
//                autoMode = true;
//            } else if (currentButtonPositionDpadDown && !previousButtonPositionDpadDown) {
//                level--;
//                autoMode = true;
//            } else if (currentButtonPositionRightBumper && !previousButtonPositionRightBumper) {
//                level = 1;
//                autoMode = true;
//            }
//        }
//        previousButtonPositionDpadDown = currentButtonPositionDpadDown;
//        previousButtonPositionDpadUp = currentButtonPositionDpadUp;
//        previousButtonPositionRightBumper = currentButtonPositionRightBumper;
//        if (autoMode) {
//            needToEstimatePos = false;
//        }
//
//        if (Math.abs(manualPower) > 0.01) {
//            autoMode = false;
//            needToEstimatePos = true;
//        }
//
//        if (autoMode) {
//            level=(int)FTCMath.clamp(0,8,level);
//            int setPoint = level * TICK_PER_STONE;
//            elevatorController.updateSetpoint(setPoint);
//            if (Math.abs(setPoint - Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition()) > 50) {
//                elevatorController.reset(Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition());
//            }
//            double power = elevatorController.update(Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition());
//            power = FTCMath.clamp(-0.5, 0.5, power);
//            Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(power);
//        } else {
//            double slowMoElevator = FTCMath.lerp(1, 0.4, FastMath.abs(gamepadService.getAnalog(2, "right_trigger")));
//            manualPower *= slowMoElevator;
//            Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(manualPower);
//        }

    }

}