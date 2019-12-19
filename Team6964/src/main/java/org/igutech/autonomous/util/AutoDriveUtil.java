package org.igutech.autonomous.util;
//
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.apache.commons.math3.util.FastMath;
import org.igutech.config.Hardware;
import org.igutech.utils.FTCMath;
import org.igutech.utils.control.BasicController;
import org.igutech.utils.control.PIDController;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class AutoDriveUtil {
//
    private AutoUtilManager manager;
    private Hardware hardware = null;

    private EncoderData encoderData;

    public Object encoderLock = new Object();

    public AutoDriveUtil(AutoUtilManager manager, Hardware hardware) {
        this.manager = manager;
        this.hardware = hardware;
        this.encoderData = new EncoderData(
                hardware.getMotors().get("frontleft").getCurrentPosition(),
                hardware.getMotors().get("frontright").getCurrentPosition(),
                hardware.getMotors().get("backleft").getCurrentPosition(),
                hardware.getMotors().get("backright").getCurrentPosition(),
                0,
                0,
                0,
                0
        );
    }

    public OffsetCorrectedEncoderData getEncoderData() {
        synchronized (encoderLock) {

            return new OffsetCorrectedEncoderData(
                    hardware.getMotors().get("frontleft").getCurrentPosition() -encoderData.frontleftoffset,
                    hardware.getMotors().get("frontright").getCurrentPosition() -encoderData.frontrightoffset,
                    hardware.getMotors().get("backleft").getCurrentPosition() -encoderData.backleftoffset,
                    hardware.getMotors().get("backright").getCurrentPosition() -encoderData.backrightoffset
            );

        }
    }

    public void resetEncoders() {
        synchronized (encoderLock) {

            encoderData = new EncoderData(
                    hardware.getMotors().get("frontleft").getCurrentPosition(),
                    hardware.getMotors().get("frontright").getCurrentPosition(),
                    hardware.getMotors().get("backleft").getCurrentPosition(),
                    hardware.getMotors().get("backright").getCurrentPosition(),

                    hardware.getMotors().get("frontleft").getCurrentPosition(),
                    hardware.getMotors().get("frontright").getCurrentPosition(),
                    hardware.getMotors().get("backleft").getCurrentPosition(),
                    hardware.getMotors().get("backright").getCurrentPosition()
            );

        }
    }

    public void drive(DriveParameters parameters) {
        setMotors(getMotorPowers(parameters));
    }

    public void driveDistance(DriveParameters parameters, double distance,
                              OffsetCorrectedEncoderData.Units units) throws InterruptedException {

        BasicController gyroController = new PIDController(0.5, 0, 0);
        gyroController.updateSetpoint(parameters.vTheta);

        while (getVD(getEncoderData().getFrontleft(),
                     getEncoderData().getFrontright(),
                     getEncoderData().getBackleft(),
                     getEncoderData().getBackright()) <
                OffsetCorrectedEncoderData.convertToTicks(distance, units)) {

            double vD = getVD(getEncoderData().getFrontleft(),
                              getEncoderData().getFrontright(),
                              getEncoderData().getBackleft(),
                              getEncoderData().getBackright());

            double targetSpeed = FTCMath.trapezoidalMotionProfile(
                    distance / 3,
                    distance / 3,
                    distance,
                    1,
                    0.2,
                    vD);

            DriveParameters newParameters = new DriveParameters(parameters);

            newParameters.setvD(targetSpeed);
            newParameters.setThetaD(gyroController.update(manager.getGyroUtil().getAngle(false).thirdAngle));

            setMotors(getMotorPowers(newParameters));
        }

        stop();
    }

    public double getVD(double m1, double m2, double m3, double m4) {
        double vTheta = getVTheta(m1, m2, m3, m4);
        return FastMath.sqrt((m1 + vTheta)*(m1 + vTheta) + (m2 + vTheta)*(m2 + vTheta));
    }

    public double getVTheta(double m1, double m2, double m3, double m4) {
        return (m4 - m1) + (m3 - m2)/4;
    }

    public DriveMotorPowers getMotorPowers(DriveParameters parameters) {
        double vD = parameters.getvD();
        double thetaD = parameters.getThetaD();
        double vTheta = parameters.getvTheta();

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
                powers.set(i, powers.get(i)/maxMag);
        }

        return new DriveMotorPowers(powers.get(0),
                powers.get(1),
                powers.get(2),
                powers.get(3));
    }

    public void setMotors(DriveMotorPowers powers) {
        hardware.getMotors().get("frontleft").setPower(powers.getFrontLeft());
        hardware.getMotors().get("frontright").setPower(powers.getFrontRight());
        hardware.getMotors().get("backleft").setPower(-powers.getBackLeft());
        hardware.getMotors().get("backright").setPower(-powers.getBackRight());
    }

    public void stop() {
        setMotors(new DriveMotorPowers(0, 0, 0, 0));
    }

    public static class DriveMotorPowers {
        private double frontLeft;
        private double frontRight;
        private double backLeft;
        private double backRight;

        public DriveMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;
        }

        public double getFrontLeft() {
            return frontLeft;
        }

        public void setFrontLeft(double frontLeft) {
            this.frontLeft = frontLeft;
        }

        public double getFrontRight() {
            return frontRight;
        }

        public void setFrontRight(double frontRight) {
            this.frontRight = frontRight;
        }

        public double getBackLeft() {
            return backLeft;
        }

        public void setBackLeft(double backLeft) {
            this.backLeft = backLeft;
        }

        public double getBackRight() {
            return backRight;
        }

        public void setBackRight(double backRight) {
            this.backRight = backRight;
        }
    }

    public static class DriveParameters {
        private double vD;
        private double thetaD;
        private double vTheta;

        /**
         * Creates driving parameters used for calculating motor powers.
         * @param vD Rate (not yaw) (-1 to 1)
         * @param thetaD Angle (radians)
         * @param vTheta Yaw rate (-1 to 1)
         */
        public DriveParameters(double vD, double thetaD, double vTheta) {
            this.vD = vD;
            this.thetaD = thetaD;
            this.vTheta = vTheta;
        }

        public DriveParameters(DriveParameters dp) {
            this.vD = dp.vD;
            this.thetaD = dp.thetaD;
            this.vTheta = dp.vTheta;
        }

        public double getvD() {
            return vD;
        }

        public void setvD(double vD) {
            this.vD = vD;
        }

        public double getThetaD() {
            return thetaD;
        }

        public void setThetaD(double thetaD) {
            this.thetaD = thetaD;
        }

        public double getvTheta() {
            return vTheta;
        }

        public void setvTheta(double vTheta) {
            this.vTheta = vTheta;
        }
    }

    public static class EncoderData {
        private int frontleft;
        private int frontright;
        private int backleft;
        private int backright;

        private int frontleftoffset;
        private int frontrightoffset;
        private int backleftoffset;
        private int backrightoffset;

        public EncoderData(int frontleft, int frontright, int backleft, int backright,
                           int frontleftoffset, int frontrightoffset, int backleftoffset, int backrightoffset) {

            this.frontleft = frontleft;
            this.frontright = frontright;
            this.backleft = backleft;
            this.backright = backright;

            this.frontleftoffset = frontleftoffset;
            this.frontrightoffset = frontrightoffset;
            this.backleftoffset = backleftoffset;
            this.backrightoffset = backrightoffset;
        }

        public int getFrontleft() {
            return frontleft;
        }

        public int getFrontright() {
            return frontright;
        }

        public int getBackleft() {
            return backleft;
        }

        public int getBackright() {
            return backright;
        }

        public int getFrontleftoffset() {
            return frontleftoffset;
        }

        public int getFrontrightoffset() {
            return frontrightoffset;
        }

        public int getBackleftoffset() {
            return backleftoffset;
        }

        public int getBackrightoffset() {
            return backrightoffset;
        }
    }

    public static class OffsetCorrectedEncoderData {

        /**
         * Wheel circumference in inches
         */
        private static final double CIRCUMFERENCE = 12.5663706144d;
        private static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
        private static final double TICKS_PER_REVOLUTION = MOTOR_CONFIG.getTicksPerRev();

        private int frontleft;
        private int frontright;
        private int backleft;
        private int backright;

        public OffsetCorrectedEncoderData(int frontleft, int frontright, int backleft, int backright) {
            this.frontleft = frontleft;
            this.frontright = frontright;
            this.backleft = backleft;
            this.backright = backright;
        }

        public int getFrontleft() {
            return frontleft;
        }

        public int getFrontright() {
            return frontright;
        }

        public int getBackleft() {
            return backleft;
        }

        public int getBackright() {
            return backright;
        }

        public static double convertToUnits(int ticks, Units units) {
            switch (units) {
                case TICKS: return ticks;
                case INCHES: return (double)ticks/TICKS_PER_REVOLUTION*CIRCUMFERENCE;
                case FEET: return convertToUnits(ticks, Units.INCHES)/12;
                case CENTIMETERS: return convertToUnits(ticks, Units.INCHES)*2.54;
                case MILLIMETERS: return convertToUnits(ticks, Units.CENTIMETERS)*10;
                case METERS: return convertToUnits(ticks, Units.CENTIMETERS) / 100;
                default:
                    throw new RuntimeException("NOT YET IMPLEMENTED");
            }
        }

        public static int convertToTicks(double other, Units units) {
            switch (units) {
                case TICKS: return (int)other;
                case INCHES: return (int) (other * TICKS_PER_REVOLUTION / CIRCUMFERENCE);
                case FEET: return convertToTicks(other * 12, Units.INCHES);
                case CENTIMETERS: return convertToTicks(other / 2.54, Units.INCHES);
                case MILLIMETERS: return convertToTicks(other / 10, Units.CENTIMETERS);
                case METERS: return convertToTicks(other * 100, Units.CENTIMETERS);
                default:
                    throw new RuntimeException("NOT YET IMPLEMENTED");
            }
        }

        public enum Units {
            TICKS,
            INCHES,
            FEET,
            MILLIMETERS,
            CENTIMETERS,
            METERS
        }
    }

}
