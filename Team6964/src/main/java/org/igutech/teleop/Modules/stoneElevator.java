package org.igutech.teleop.Modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;

import org.igutech.utils.control.*;

import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;
import org.igutech.utils.FTCMath;

@Config
public class stoneElevator extends Module {

    private GamepadService gamepadService;
    public static double p = 0.03;

    private static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);

    /**
     * Window size in wheel rotations
     */
    private final double WINDOW_SIZE = 2.7; // Actual = 4.22
    private double windowPosition = 0;

    private final double MAX_SPEED_LIMIT = 0.4;

    private int liftPosition;
    private int liftEncoderOffset=4;
    private static final int TICK_PER_STONE = 5000;

    private boolean toggleA = false;
    private boolean toggleY = false;
    private boolean toggleX = false;
    private boolean previousButtonPositionA = false;
    private boolean previousButtonPositionY = false;
    private boolean previousButtonPositionX = false;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public stoneElevator() {
        super(700, "stoneElevator");
    }

    PControllers PID = new PControllers(p);

    @Override
    public void init() {
        gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
        liftPosition = 0;

    }

    @Override
    public void start() {
        liftEncoderOffset = Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition();

    }

    @Override
    public void loop() {

        double elevatorPower = gamepadService.getAnalog(2, "right_stick_y");

        double slowMoElevator = gamepadService.getAnalog(2, "right_trigger");
        double vdMultElevator = FTCMath.lerp(1, 0.4, FastMath.abs(slowMoElevator));

        elevatorPower *= vdMultElevator;

        //liftPosition = toRotations(Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition(), liftEncoderOffset);
        liftPosition = Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition();

//        if(Teleop.getInstance().getHardware().getTouchSensors().get("elevator_switch").getState())
//        Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(elevatorPower);
//        else if(Teleop.getInstance().getHardware().getTouchSensors().get("elevator_switch").getState()==false & elevatorPower<=0
//        )


        if (elevatorPower != 0.0) {
            Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(elevatorPower);
        } else {
            PID.pControl(Teleop.getInstance().getHardware().getMotors().get("stoneElevator"),
                    Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition(),
                    Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition());
        }
        //b rotation,a grabber, x for transfer
        boolean currentButtonPositionA = gamepadService.getDigital(2, "a");

        if (currentButtonPositionA && !previousButtonPositionA) {
            toggleA = !toggleA;
            if (toggleA) {
                Teleop.getInstance().getHardware().getServos().get("GrabberServo").setPosition(0.3);
            }
            if (!toggleA) {
                Teleop.getInstance().getHardware().getServos().get("GrabberServo").setPosition(0.1);
            }
        }
        previousButtonPositionA = currentButtonPositionA;

        boolean currentButtonPositionY = gamepadService.getDigital(2, "y");

        if (currentButtonPositionY && !previousButtonPositionY) {
            toggleY = !toggleY;
            if (toggleY) {
                Teleop.getInstance().getHardware().getServos().get("RotationServo").setPosition(0.95);
            }
            if (!toggleY) {
                Teleop.getInstance().getHardware().getServos().get("RotationServo").setPosition(0.28);
            }
        }
        previousButtonPositionY = currentButtonPositionY;
        Teleop.getInstance().telemetry.addData("elevator",liftPosition);

        dashboardTelemetry.addData("liftpos",liftPosition);
        dashboardTelemetry.update();


    }

    private double toRotations(int ticks, int offset) {
        return (double) (ticks - offset) / MOTOR_CONFIG.getTicksPerRev();
    }

}
