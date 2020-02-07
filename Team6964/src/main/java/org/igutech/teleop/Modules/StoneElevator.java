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

    private ElevatorState elevatorState = ElevatorState.OFF;
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

    private boolean previousButtonPositionLeftBumper = false;
    private boolean currentButtonPositionLeftBumper = false;
    private boolean toggleDpadLeftBumper = false;

    private boolean previousButtonPositionBack = false;
    private boolean currentButtonPositionBack = false;
    private boolean toggleBack = false;

    private boolean autoMode = false;
    private boolean needToEstimatePos = false;

    private int level = 0;
    public static int TICK_PER_STONE = 240;

    public static double p = 0.02;
    public static double i = 0.00;
    public static double d = 0.0005;
    private int lastLevel = 0;
    private boolean automaticMode = true;

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
        currentButtonPositionLeftBumper = gamepadService.getDigital(2, "left_bumper");
        currentButtonPositionBack = gamepadService.getDigital(2, "back");

        if (currentButtonPositionDpadUp && !previousButtonPositionDpadUp) {
            elevatorState = ElevatorState.RISE;
            lastLevel++;

        }
        if (currentButtonPositionDpadDown && !previousButtonPositionDpadDown) {
            elevatorState = ElevatorState.DOWN;
            level = 0;
        }
        if (currentButtonPositionRightBumper && !previousButtonPositionRightBumper) {
            elevatorState = ElevatorState.DEFAULT;

        }
        if (currentButtonPositionLeftBumper && !previousButtonPositionLeftBumper) {
            lastLevel = 0;
        }
        if (currentButtonPositionBack && !previousButtonPositionBack) {
            automaticMode = !automaticMode;
        }

        previousButtonPositionDpadDown = currentButtonPositionDpadDown;
        previousButtonPositionDpadUp = currentButtonPositionDpadUp;
        previousButtonPositionRightBumper = currentButtonPositionRightBumper;
        previousButtonPositionLeftBumper = currentButtonPositionLeftBumper;
        previousButtonPositionBack = currentButtonPositionBack;

        if (automaticMode) {
            switch (elevatorState) {
                case RISE:
                    level = lastLevel;
                    autoMode = true;
                    break;
                case DOWN:
                    level = 0;
                    autoMode = true;
                    if (Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition() < startPos + 10) {
                        autoMode = false;
                        elevatorState = ElevatorState.OFF;
                    }
                    break;
                case DEFAULT:
                    if(Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition()<800){
                        level = 5;
                    }
                    autoMode = true;
                    if (reset) {
                        time = System.currentTimeMillis();
                        reset = false;
                    }
                    Teleop.getInstance().getHardware().getServos().get("TransferServo").setPosition(0.43);

                    if (Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition() > 360) {
                        Teleop.getInstance().getHardware().getServos().get("RotationServo").setPosition(0.28);

                        if ((System.currentTimeMillis() - time) > 2000) {
                            elevatorState = ElevatorState.DOWN;
                            reset = true;
                        }
                    }
                    break;
                case OFF:
                    autoMode = false;
                    break;
            }
        }

        if (Math.abs(manualPower) > 0.01) {
            autoMode = false;
            elevatorState = ElevatorState.OFF;
        }

        if (autoMode) {
            level = (int) FTCMath.clamp(0, 8, level);
            int setPoint = (int) startPos + (level * TICK_PER_STONE);
            elevatorController.updateSetpoint(setPoint);
            if (Math.abs(setPoint - Teleop.getInstance().getHardware().getMotors()
                    .get("stoneElevator").getCurrentPosition()) > 50) {
                elevatorController.reset(Teleop.getInstance().getHardware()
                        .getMotors().get("stoneElevator").getCurrentPosition());
            }
            double power = elevatorController.update(Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition());
            power = FTCMath.clamp(-0.75, 0.75, power);
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
        DOWN,
        OFF
    }

}