package org.igutech.teleop.Modules;

import org.apache.commons.math3.util.FastMath;
import org.igutech.teleop.Module;
import org.igutech.teleop.Teleop;
import org.igutech.utils.FTCMath;
import org.igutech.utils.control.BasicController;
import org.igutech.utils.control.PIDController;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;


public class DriveTrain extends Module {

    private GamepadService gamepadService;
    private GyroService gyroService;
    private BasicController gyroController;

    private final double kP = 0.02;
    private final double kI = 0.006;
    private final double kD = 0;

    private double yawTarget = 0;
    private final double YAW_RATE = 4;
    private double pidAdjustmentOffset = 0.001;

    private final boolean PID_STABILIZATION_ENABLED = false;
    private final boolean PID_ADJUSTMENT_ENABLED = false;

    public DriveTrain() {
        super(1000, "Drivetrain"); //Highest priority
    }

    @Override
    public void init() {
        gamepadService = (GamepadService) Teleop.getInstance().getService("GamepadService");
        gyroService = (GyroService) Teleop.getInstance().getService("GyroService");
        if (PID_STABILIZATION_ENABLED)
            gyroController = new PIDController(kP, kI, kD);
    }

    @Override
    public void start() {
        if (PID_STABILIZATION_ENABLED)
            gyroController.init();
    }

    @Override
    public void loop() {

        if (PID_STABILIZATION_ENABLED) {
            if (gamepadService.getDigital(1, "b")) {
                gyroController = new PIDController(kP, kI, kD);
                gyroController.init();
                yawTarget = gyroService.getIntegratedAngle();
            }
        }

        double vD = FastMath.hypot(gamepadService.getAnalog(1, "right_stick_x"),
                -gamepadService.getAnalog(1, "right_stick_y"));
        double thetaD = Math.atan2(-gamepadService.getAnalog(1, "right_stick_x"),
                gamepadService.getAnalog(1, "right_stick_y")) + FastMath.PI / 4;

        double vTheta;
        if (PID_STABILIZATION_ENABLED) {
            yawTarget += -gamepadService.getAnalog(1, "left_stick_x") * YAW_RATE;
            gyroController.updateSetpoint(yawTarget);
            vTheta = gyroController.update(gyroService.getIntegratedAngle());
        } else {
            vTheta = -gamepadService.getAnalog(1, "left_stick_x");
        }


        //if (PID_STABILIZATION_ENABLED)
        //Log.d("DRIVETRAIN", "\t" + yawTarget + "\t" + vTheta + "\t" + gyroService.getIntegratedAngle());


        double slowMo = gamepadService.getAnalog(1, "right_trigger");
        double vdMult = FTCMath.lerp(1, 0.4, FastMath.abs(slowMo));
        double vThetaMult = FTCMath.lerp(.8, 0.15, FastMath.abs(slowMo));
        vD *= vdMult;
        vTheta *= vThetaMult;

//        Teleop.getInstance().telemetry.addData("slowMo", slowMo);
//        Teleop.getInstance().telemetry.addData("vdMult", vdMult);
//        Teleop.getInstance().telemetry.addData("vThetaMult", vThetaMult);

        double frontLeft = vD * FastMath.sin(-thetaD) - vTheta;
        double frontRight = vD * FastMath.cos(-thetaD) - vTheta;
        double backLeft = vD * FastMath.cos(-thetaD) + vTheta;
        double backRight = vD * FastMath.sin(-thetaD) + vTheta;


        List<Double> powers = Arrays.asList(frontLeft, frontRight, backLeft, backRight);
        double minPower = Collections.min(powers);
        double maxPower = Collections.max(powers);
        double maxMag = FastMath.max(FastMath.abs(minPower), FastMath.abs(maxPower));

        if (maxMag > 1.0) {
            for (int i = 0; i < powers.size(); i++)
                powers.set(i, powers.get(i) / maxMag);
        }

        for (int i = 0; i < powers.size(); i++) {
            //powers.set(i, powers.get(i)*0.4);
        }

//        if (!gamepadService.getDigital(2, "dpad_left")) {}

        Teleop.getInstance().getHardware().getMotors().get("frontleft").setPower(powers.get(0));
        Teleop.getInstance().getHardware().getMotors().get("frontright").setPower(powers.get(1));
        Teleop.getInstance().getHardware().getMotors().get("backleft").setPower(-powers.get(2));
        Teleop.getInstance().getHardware().getMotors().get("backright").setPower(-powers.get(3));


//        Teleop.getInstance().telemetry.addData("FrontLeft ", powers.get(0));
//        Teleop.getInstance().telemetry.addData("FrontRight",powers.get(1));
//        Teleop.getInstance().telemetry.addData("BackLeft", -powers.get(2));
        Teleop.getInstance().telemetry.addData("BackRight",Teleop.getInstance().getHardware().getMotors().get("backright").getCurrentPosition());
        Teleop.getInstance().telemetry.addData("tick", Teleop.getInstance().getHardware().getMotors().get("stoneElevator").getCurrentPosition());

        if (PID_STABILIZATION_ENABLED && PID_ADJUSTMENT_ENABLED) {
            PIDController controller = (PIDController) gyroController;
            if (gamepadService.getDigital(1, "dpad_up"))
                controller.setkP(controller.getkP()+pidAdjustmentOffset);
            if (gamepadService.getDigital(1, "dpad_down"))
                controller.setkP(controller.getkP()-pidAdjustmentOffset);
            if (gamepadService.getDigital(1, "dpad_right"))
                controller.setkI(controller.getkI()+pidAdjustmentOffset);
            if (gamepadService.getDigital(1, "dpad_left"))
                controller.setkI(controller.getkI()-pidAdjustmentOffset);
            if (gamepadService.getDigital(1, "left_bumper"))
                controller.setkD(controller.getkD()+pidAdjustmentOffset);
            if (gamepadService.getAnalog(1, "left_trigger") > 0.7)
                controller.setkD(controller.getkD()-pidAdjustmentOffset);
            if (gamepadService.getDigital(1, "y"))
                pidAdjustmentOffset += 0.001;
            if (gamepadService.getDigital(1, "a"))
                pidAdjustmentOffset -= 0.001;
            gyroController = controller;

            Teleop.getInstance().telemetry.addData("kP", ((PIDController)gyroController).getkP());
            Teleop.getInstance().telemetry.addData("kI", ((PIDController)gyroController).getkI());
            Teleop.getInstance().telemetry.addData("kD", ((PIDController)gyroController).getkD());
            Teleop.getInstance().telemetry.addData("AdjustmentValue", pidAdjustmentOffset);
        }
    }
}
