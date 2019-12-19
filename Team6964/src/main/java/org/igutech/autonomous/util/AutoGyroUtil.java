package org.igutech.autonomous.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.igutech.config.Hardware;

public class AutoGyroUtil {

    private BNO055IMU imu;
    private Orientation angles = null;

    public AutoGyroUtil(AutoUtilManager manager, Hardware hardware) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardware.getHardwareMap().get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void invalidateCache() {
        angles = null;
    }

    public Orientation getAngle(boolean useCache) {
        if (!useCache || angles == null) {
            angles = imu.getAngularOrientation();
        }

        return angles;
    }
}
