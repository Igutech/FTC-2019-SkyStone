package org.igutech.teleop.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.FastMath;
import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;
import org.igutech.utils.control.PIDController;

import org.igutech.utils.FTCMath;

@Config
public class ConceptStoneElevator extends Module {

    private GamepadService gamepadService;
    private ElapsedTime runtime = new ElapsedTime();
    public boolean reset=true;
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

    public static double p = 0.02;
    public static double i = 0.00;
    public static double d = 0.0005;

    private PIDController elevatorController = new PIDController(p, i, d);

    public ConceptStoneElevator() {
        super(710, "ConceptStoneElevator");
    }

    public void init() {
        gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
    }

    public void start() {
        startPos = Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition();
        runtime.reset();
    }

    public void loop() {
        //Teleop.getInstance().telemetry.addData("time",runtime.milliseconds());
        Teleop.getInstance().telemetry.addData("Current tick",
                Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition());
        manualPower = -1 * gamepadService.getAnalog(2, "right_stick_y");
        currentButtonPositionDpadUp = gamepadService.getDigital(2, "dpad_up");
        currentButtonPositionDpadDown = gamepadService.getDigital(2, "dpad_down");
        currentButtonPositionRightBumper = gamepadService.getDigital(2, "right_bumper");

        if (elevatorState == ElevatorState.RISE) {
            autoMode = true;
        } else if (elevatorState == ElevatorState.DOWN) {
            level = 0;
            autoMode = true;

        } else if (elevatorState == ElevatorState.DEFAULT) {
            autoMode = true;
            if(reset)
            {
                time=System.currentTimeMillis();
                reset=false;
            }
            if (Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition() > 1000) {
                Teleop.getInstance().getHardware().getServos().get("RotationServo").setPosition(0.28);
                Teleop.getInstance().telemetry.addData("first",time);

                Teleop.getInstance().telemetry.addData("timeData",System.currentTimeMillis());
                Teleop.getInstance().telemetry.addData("t",System.currentTimeMillis()-time);

                if((System.currentTimeMillis()-time)>2500){
                    elevatorState = ElevatorState.DOWN;
                    reset=true;
                }

            }
        }

        if (currentButtonPositionDpadUp && !previousButtonPositionDpadUp) {
            elevatorState = ElevatorState.RISE;
            level++;

        } else if (currentButtonPositionDpadDown && !previousButtonPositionDpadDown) {
            elevatorState = ElevatorState.DOWN;
            level = 0;

        } else if (currentButtonPositionRightBumper && !previousButtonPositionRightBumper) {
            elevatorState = ElevatorState.DEFAULT;
            level = 5;

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
        DOWN,
        OFF
    }

}