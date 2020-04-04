package org.igutech.teleop.Modules;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.igutech.teleop.Service;
import org.igutech.teleop.Teleop;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class BulkRead extends Service {
    private HardwareMap hardwareMap;
    private RevBulkData bulkData;
    private AnalogInput a0, a1, a2, a3;
    private DigitalChannel d0, d1, d2, d3, d4, d5, d6, d7;
    private ExpansionHubMotor motor0, motor1, motor2, motor3;
    private ExpansionHubEx expansionHub;

    public BulkRead() {
        super("BulkRead");
    }

    @Override
    public void init() {
        expansionHub = Teleop.getInstance().getHardware().getHardwareMap().get(ExpansionHubEx.class, "Expansion Hub 2");
        motor0 = (ExpansionHubMotor) Teleop.getInstance().getHardware().getMotors().get("frontleft");
        motor1 = (ExpansionHubMotor) Teleop.getInstance().getHardware().getMotors().get("frontright");
        motor2 = (ExpansionHubMotor) Teleop.getInstance().getHardware().getMotors().get("backleft");
        motor3 = (ExpansionHubMotor) Teleop.getInstance().getHardware().getMotors().get("backright");

        loop();
    }

    @Override
    public void loop() {
        bulkData = expansionHub.getBulkInputData();
    }

    public int getMotorZeroTicks() {
        return bulkData.getMotorCurrentPosition(motor0);
    }

    public int getMotorOneTicks() {
        return bulkData.getMotorCurrentPosition(motor1);
    }

    public int getMotorTwoTicks() {
        return bulkData.getMotorCurrentPosition(motor2);
    }

    public int getMotorThreeTicks() {
        return bulkData.getMotorCurrentPosition(motor3);
    }
}
