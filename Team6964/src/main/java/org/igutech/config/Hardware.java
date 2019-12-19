package org.igutech.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

public class Hardware {

    private HardwareMap hardwareMap;
    private Map<String, DcMotor> motors;
    private Map<String, DcMotorImplEx> DcMotorImplExMotor;

    private Map<String, Servo> servos;
    private Map<String, DigitalChannel> touchSensors;

    /**
     * Initialize the hardware object
     *
     * @param hardwareMap Hardware map provided by FIRST
     */
    public Hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.motors = new HashMap<>();
        this.servos = new HashMap<>();
        this.touchSensors = new HashMap<>();

        motors.put("frontleft", hardwareMap.dcMotor.get("frontleft"));
        motors.put("frontright", hardwareMap.dcMotor.get("frontright"));
        motors.put("backleft", hardwareMap.dcMotor.get("backleft"));
        motors.put("backright", hardwareMap.dcMotor.get("backright"));
        motors.put("left_intake", hardwareMap.dcMotor.get("left_intake"));
        motors.put("right_intake", hardwareMap.dcMotor.get("right_intake"));
        motors.put("stoneElevator",hardwareMap.dcMotor.get("stoneElevator"));


        motors.get("frontright").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.get("frontleft").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.get("backright").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.get("backleft").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.get("left_intake").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.get("right_intake").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.get("stoneElevator").setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motors.get("frontright").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get("frontleft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get("backright").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get("backleft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get("left_intake").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get("right_intake").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motors.get("stoneElevator").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        servos.put("FoundationServo_left", hardwareMap.servo.get("FoundationServo_left"));
        servos.put("FoundationServo_right", hardwareMap.servo.get("FoundationServo_right"));
        servos.put("GrabberServo", hardwareMap.servo.get("GrabberServo"));
        servos.put("RotationServo", hardwareMap.servo.get("RotationServo"));
        servos.put("TransferServo", hardwareMap.servo.get("TransferServo"));
        servos.put("CapServo", hardwareMap.servo.get("CapServo"));

//        touchSensors.put("elevator_switch", hardwareMap.get(DigitalChannel.class, "elevator_switch"));
//        touchSensors.get("elevator_switch").setMode(DigitalChannel.Mode.INPUT);

    }

    /**
     * Get the raw hardware map object provided by FIRST
     *
     * @return
     */
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    /**
     * Get a map of motors
     *
     * @return Map of motor name/motor
     */
    public Map<String, DcMotor> getMotors() {
        return motors;
    }

    /**
     * Get a map of servos
     *
     * @return
     */
    public Map<String, Servo> getServos() {
        return servos;
    }

    /**
     * Get a map of touch sensors
     *
     * @return Map of touch sensor name/Digital Channel
     */
    public Map<String, DigitalChannel> getTouchSensors() {
        return touchSensors;
    }
}
