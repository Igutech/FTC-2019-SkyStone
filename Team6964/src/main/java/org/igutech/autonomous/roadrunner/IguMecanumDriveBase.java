package org.igutech.autonomous.roadrunner;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.igutech.autonomous.util.AutoDriveUtil;
import org.igutech.autonomous.util.AutoUtilManager;
import org.igutech.utils.FTCMath;
import org.igutech.utils.control.PIDController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.igutech.autonomous.AutoPrograms.RedRoadRunnerDepot.manager;

public class IguMecanumDriveBase extends MecanumDriveBase {

    private List<DcMotorEx> motors;
    private AutoUtilManager utils;
    private PIDController elevatorController = new PIDController(0.015, 0.0, 0.0002);
    public static final boolean RUN_USING_ENCODER = true;
    private boolean enableElevator = false;
    private ElevatorState elevatorState = ElevatorState.OFF;
    int startPos = manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition();

    public IguMecanumDriveBase(AutoUtilManager utils) {
        super();


        this.utils = utils;

        motors = Arrays.asList(
                (DcMotorEx) utils.getHardware().getMotors().get("frontleft"),
                (DcMotorEx) utils.getHardware().getMotors().get("backleft"),
                (DcMotorEx) utils.getHardware().getMotors().get("backright"),
                (DcMotorEx) utils.getHardware().getMotors().get("frontright")
        );

        motors.get(0).setDirection(DcMotorSimple.Direction.FORWARD);
        motors.get(1).setDirection(DcMotorSimple.Direction.FORWARD);
        motors.get(2).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(3).setDirection(DcMotorSimple.Direction.REVERSE);

        //TODO: Set PID coefficients
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        this.setLocalizer(new MecanumDrive.MecanumLocalizer(this, true));

    }

    @Override
    public void update() {
        super.update();
        updateElevator();
    }


    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = motors.get(0).getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        //Log.d("MECANUM_DRIVE", v + " " + v1 + " " + v2 + " " + v3);
        motors.get(0).setPower(v);
        motors.get(1).setPower(v1);
        motors.get(2).setPower(v2);
        motors.get(3).setPower(v3);
    }


    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(AutoDriveUtil.OffsetCorrectedEncoderData.convertToUnits(utils.getDriveUtil().getEncoderData().getFrontleft(), AutoDriveUtil.OffsetCorrectedEncoderData.Units.INCHES));
        wheelPositions.add(AutoDriveUtil.OffsetCorrectedEncoderData.convertToUnits(utils.getDriveUtil().getEncoderData().getBackleft(), AutoDriveUtil.OffsetCorrectedEncoderData.Units.INCHES));
        wheelPositions.add(AutoDriveUtil.OffsetCorrectedEncoderData.convertToUnits(utils.getDriveUtil().getEncoderData().getBackright(), AutoDriveUtil.OffsetCorrectedEncoderData.Units.INCHES));
        wheelPositions.add(AutoDriveUtil.OffsetCorrectedEncoderData.convertToUnits(utils.getDriveUtil().getEncoderData().getFrontright(), AutoDriveUtil.OffsetCorrectedEncoderData.Units.INCHES));
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(AutoDriveUtil.OffsetCorrectedEncoderData.convertToUnits((int) motor.getVelocity(), AutoDriveUtil.OffsetCorrectedEncoderData.Units.INCHES));
        }
        return wheelVelocities;
    }

    @Override
    public double getRawExternalHeading() {
        return utils.getGyroUtil().getAngle(false).firstAngle;
    }

    public void setElevatorTick(int tick) {
        enableElevator = true;
        elevatorController.updateSetpoint(tick);
    }

    public void updateElevator() {
//        if(elevatorState==ElevatorState.DOWN &&
//                manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition()>startPos-20 ){
//            changeElevatorState(ElevatorState.OFF);
//        }
        double power = elevatorController.update(manager.getHardware().getMotors().get("stoneElevator").getCurrentPosition());
        power = FTCMath.clamp(-0.5, 0.5, power);
        if (elevatorState == ElevatorState.OFF) {
            enableElevator = false;
            power = 0.0;
        }
        System.out.println("state: " + elevatorState);
        System.out.println("power: " + power);
        manager.getHardware().getMotors().get("stoneElevator").setPower(power);
    }

    public synchronized void changeElevatorState(ElevatorState state) {
        elevatorState = state;
        switch (elevatorState) {
            case DOWN:
                setElevatorTick(-20);
                break;
            case UP:
                setElevatorTick(-700);
                break;
            case OFF:

                enableElevator = false;
                break;
            default:
                enableElevator = false;
                break;
        }

    }

    public enum ElevatorState {
        DOWN,
        UP,
        OFF

    }
}
