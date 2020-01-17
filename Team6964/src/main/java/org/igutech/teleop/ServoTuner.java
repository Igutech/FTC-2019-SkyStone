package org.igutech.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class ServoTuner extends LinearOpMode {

    public static double Pos=0.4;
    //netrual 0.58
    //0.8
    public static String name="CapServo";
    @Override
    public void runOpMode()
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        Servo ServoThing=hardwareMap.servo.get(name);
        waitForStart();
       while(opModeIsActive())
       {
           ServoThing.setPosition(Pos);
           dashboardTelemetry.addData("Position",Pos);
           dashboardTelemetry.update();
       }

    }
}
