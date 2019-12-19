package org.igutech.teleop.Modules;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.igutech.config.Hardware;
import org.igutech.teleop.Service;

public class GyroService extends Service {

    private BNO055IMU imu;
    private Orientation angles;

    private long lastReading = -1;
    private long loopCounter = 0;

    private double lastIntegrationReading = 0;
    private int turns = 0;

    public GyroService(Hardware hardware) {
        super("GyroService");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardware.getHardwareMap().get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        newLoop();
    }

    /**
     * Invalidates cache
     */
    public void newLoop() {
        loopCounter++;
    }

    public Orientation getAngles() {
        if (loopCounter == lastReading) return angles;

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        lastReading = loopCounter;

        //Log.d("GyroService", angles.firstAngle + " " + lastIntegrationReading + " " + turns);
        if (Math.abs(angles.firstAngle - lastIntegrationReading) > 180) { // We made a full turn around
            turns += angles.firstAngle < lastIntegrationReading ? 1 : -1;
        }

        lastIntegrationReading = angles.firstAngle;
        return angles;
    }

    public double getIntegratedAngle() {
        return getAngles().firstAngle + turns * 360;
    }
}
