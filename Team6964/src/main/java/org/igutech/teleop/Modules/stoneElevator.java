package org.igutech.teleop.Modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.igutech.autonomous.roadrunner.Elevator;
import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;
import org.igutech.utils.FTCMath;
import org.igutech.utils.control.PControllers;
import org.igutech.config.Hardware.*;

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
    private int liftEncoderOffset;
    private static final int TICK_PER_STONE = 5000;

    private boolean toggleA = false;
    private boolean toggleB = false;
    private boolean toggleX = false;
    private boolean previousButtonPositionA = false;
    private boolean previousButtonPositionB = false;
    private boolean previousButtonPositionX = false;
    double height = 0.0;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    Elevator elevator = new Elevator(Teleop.getInstance().hardwareMap);


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
        Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(elevatorPower);
//        else if(Teleop.getInstance().getHardware().getTouchSensors().get("elevator_switch").getState()==false & elevatorPower<=0
//        )
       /* leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(5000);
        leftMotor.setPower(0.25);*/



        //b rotation,a grabber, x for transfer
//        boolean currentButtonPositionA = gamepadService.getDigital(2, "a");
//
//        if (currentButtonPositionA && !previousButtonPositionA) {
//          height=height+10;
//        }
//        previousButtonPositionA = currentButtonPositionA;
//        if (elevatorPower != 0.0) {
//            Teleop.getInstance().getHardware().getMotors().get("stoneElevator").setPower(elevatorPower);
//        }else if(gamepadService.getDigital(1,"dpad_up"))
//        {
//            height=height+10;
//            Teleop.getInstance().getHardware().getElevator().get("elevator").setHeight(height);
//
//            Teleop.getInstance().telemetry.addData("dpad",height);
//        }
//        else if(gamepadService.getDigital(1,"dpad_down"))
//        {
//            height=height-10;
//
//            Teleop.getInstance().getHardware().getElevator().get("elevator").setHeight(0);
//            Teleop.getInstance().telemetry.addData("dpad","pressed");
//            Teleop.getInstance().getHardware().getElevator().get("elevator").update();
//
//        }
//        Teleop.getInstance().getHardware().getElevator().get("elevator").update();

        Teleop.getInstance().telemetry.addData("height",height);
        Teleop.getInstance().telemetry.addData("encoder",liftPosition);



        boolean currentButtonPositionB = gamepadService.getDigital(2, "y");

        if (currentButtonPositionB && !previousButtonPositionB) {
            toggleB = !toggleB;
            if (toggleB) {
                Teleop.getInstance().getHardware().getServos().get("RotationServo").setPosition(0.95);
            }
            if (!toggleB) {
                Teleop.getInstance().getHardware().getServos().get("RotationServo").setPosition(0.28);
            }
        }
        previousButtonPositionB = currentButtonPositionB;


//        dashboardTelemetry.addData("liftpos",liftPosition);
//        dashboardTelemetry.update();


    }

    private double toRotations(int ticks, int offset) {
        return (double) (ticks - offset) / MOTOR_CONFIG.getTicksPerRev();
    }

}
