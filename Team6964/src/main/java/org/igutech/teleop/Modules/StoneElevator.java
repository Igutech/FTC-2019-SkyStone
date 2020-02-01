package org.igutech.teleop.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.FastMath;
import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;
import org.igutech.utils.control.PIDController;

import org.igutech.utils.FTCMath;

@Config
public class StoneElevator extends Module {

    private GamepadService gamepadService;
    private ElapsedTime runtime = new ElapsedTime();
    public boolean reset = true;
    public double time;

    private ElevatorState elevatorState;
    private double manualPower = 0.0;
    private double startPos;

    private boolean previousButtonPositionDpadUp = false;
    private boolean currentButtonPositionDpadUp = false;
    private boolean toggleDpadUp = false;


    private boolean previousButtonPositionDpadDown = false;
    private boolean currentButtonPositionDpadDown = false;
    private boolean toggleDpadDown = false;

    private boolean previousButtonPositionRightBumper = false;
    private boolean currentButtonPositionRightBumper = false;
    private boolean toggleDpadRightBumper = false;

    private boolean autoMode = false;
    private boolean needToEstimatePos = false;

    private int level = 0;
    private final int TICK_PER_STONE = 250;
    private final int SAFE_ROTATION_TICK = 1000;

    public static double p = 0.02;
    public static double i = 0.00;
    public static double d = 0.0005;

    private PIDController elevatorController = new PIDController(p, i, d);

    public StoneElevator() {
        super(710, "StoneElevator");
    }

    public void init() {
        gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
    }

    public void start() {
        startPos = Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition();
        runtime.reset();
    }

    public void loop() {
        manualPower = -1 * gamepadService.getAnalog(2, "right_stick_y");
        currentButtonPositionDpadUp = gamepadService.getDigital(2, "dpad_up");
        currentButtonPositionDpadDown = gamepadService.getDigital(2, "dpad_down");
        currentButtonPositionRightBumper = gamepadService.getDigital(2, "right_bumper");

        switch (elevatorState) {
            case RISE:
                level++;
                autoMode = true;
                elevatorState = ElevatorState.MOVING;
                break;
            case DOWN:
                level = 0;
                autoMode = true;
                elevatorState = ElevatorState.MOVING;
                break;
            case DEFAULT:
                level = 5;
                autoMode = true;
                if (reset) {
                    time = System.currentTimeMillis();
                    reset = false;
                }
                if (Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition() > SAFE_ROTATION_TICK) {
                    Teleop.getInstance().getHardware().getServos().get("RotationServo").setPosition(0.28);
                    if ((System.currentTimeMillis() - time) > 2500) {
                        elevatorState = ElevatorState.DOWN;
                        reset = true;
                    }
                }
                break;
            case MOVING:
                if (level == 0 && Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition() < 10) {
                    elevatorState = ElevatorState.OFF;
                }
            case OFF:
                autoMode = false;
                break;

        }


        if (currentButtonPositionDpadUp && !previousButtonPositionDpadUp) {
            elevatorState = ElevatorState.RISE;

        } else if (currentButtonPositionDpadDown && !previousButtonPositionDpadDown) {
            elevatorState = ElevatorState.DEFAULT;

        } else if (currentButtonPositionRightBumper && !previousButtonPositionRightBumper) {
            elevatorState = ElevatorState.DOWN;
        }
        previousButtonPositionDpadDown = currentButtonPositionDpadDown;
        previousButtonPositionDpadUp = currentButtonPositionDpadUp;
        previousButtonPositionRightBumper = currentButtonPositionRightBumper;

        if (Math.abs(manualPower) > 0.01) {
            autoMode = false;
        }

        if (autoMode) {
            level = (int) FTCMath.clamp(0, 8, level);
            int setPoint = level * TICK_PER_STONE;
            elevatorController.updateSetpoint(setPoint);
            if (Math.abs(setPoint - Teleop.getInstance().getHardware().getMotors()
                    .get("stoneElevator").getCurrentPosition()) > 50) {
                elevatorController.reset(Teleop.getInstance().getHardware()
                        .getMotors().get("stoneElevator").getCurrentPosition());
            }
            double power = elevatorController.update(Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition());
            power = FTCMath.clamp(-0.5, 0.5, power);
            Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(power);
        } else {
            double slowMoElevator = FTCMath.lerp(1, 0.4, FastMath.abs(gamepadService.getAnalog(2, "right_trigger")));
            manualPower *= slowMoElevator;
            Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(manualPower);
        }

    }

    private enum ElevatorState {
        DEFAULT,
        RISE,
        MOVING,
        DOWN,
        OFF
    }

}