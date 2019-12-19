package org.igutech.autonomous.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.igutech.config.Hardware;

public class AutoUtilManager {

    private Hardware hardware;
    private AutoDriveUtil driveUtil;
    private AutoCVUtil cvUtil;
    //private AutoConfigUtil configUtil;
    private AutoGyroUtil gyroUtil;

    public AutoUtilManager(HardwareMap hardwareMap, String name) {
        this.hardware = new Hardware(hardwareMap);
        this.driveUtil = new AutoDriveUtil(this, hardware);
        this.cvUtil = new AutoCVUtil(this, hardware);
        //this.configUtil = new AutoConfigUtil(this, name);
        this.gyroUtil = new AutoGyroUtil(this, hardware);
    }

    public Hardware getHardware() {
        return hardware;
    }

    public AutoDriveUtil getDriveUtil() {
        return driveUtil;
    }

//    public AutoConfigUtil getConfigUtil() {
//        return configUtil;
//    }

    public AutoGyroUtil getGyroUtil() {
        return gyroUtil;
    }

    public AutoCVUtil getCvUtil() {
        return cvUtil;
    }
}