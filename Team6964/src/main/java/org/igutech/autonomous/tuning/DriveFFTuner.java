package org.igutech.autonomous.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.tuning.AccelRegression;
import com.acmerobotics.roadrunner.tuning.RampRegression;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.igutech.autonomous.roadrunner.IguMecanumDriveBase;
import org.igutech.autonomous.util.AutoUtilManager;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name="DriveFFTuner", group="igutech")
public class DriveFFTuner extends LinearOpMode {

    public static final double MAX_POWER = 0.7;
    public static final double DISTANCE = 100; // use to be 4*12
    @Override
    public void runOpMode() {
        AutoUtilManager manager = new AutoUtilManager(hardwareMap, "DriveFFTuner");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        manager.getDriveUtil().resetEncoders();
        IguMecanumDriveBase drive = new IguMecanumDriveBase(manager);



        NanoClock clock = NanoClock.system();

        telemetry.log().add("Press play to begin the feedforward tuning routine");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.log().clear();
        telemetry.log().add("Would you like to fit kStatic?");
        telemetry.log().add("Press (A) for yes, (B) for no");
        telemetry.update();

        boolean fitIntercept = false;
        while (!isStopRequested()) {
            if (gamepad1.a) {
                fitIntercept = true;
                while (!isStopRequested() && gamepad1.a) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (!isStopRequested() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        telemetry.clearAll();
        telemetry.addLine(Misc.formatInvariant(
                "Place your robot on the field with at least %.2f in of room in front", DISTANCE));
        telemetry.addLine("Press (A) to begin");
        telemetry.update();

        while (!isStopRequested() && !gamepad1.a) {
            idle();
        }
        while (!isStopRequested() && gamepad1.a) {
            idle();
        }


        telemetry.clearAll();
        telemetry.addLine("Running...");
        telemetry.update();

        MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);

        double maxVel = MOTOR_CONFIG.getMaxRPM() * 2 * Math.PI * 2 / 60.0;
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);

        double startTime = clock.seconds();
        RampRegression rampRegression = new RampRegression();
        List<Double> timeSamples = new ArrayList<>();
        List<Double> powerSamples = new ArrayList<>();
        List<Double> positionSamples = new ArrayList<>();

        drive.setPoseEstimate(new Pose2d());
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            rampRegression.add(elapsedTime, drive.getPoseEstimate().getX(), power);

            drive.setDrivePower(new Pose2d(power, 0.0, 0.0));
            drive.updatePoseEstimate();
        }
        drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

        RampRegression.RampResult rampResult = rampRegression.fit(fitIntercept);

        rampRegression.save(LoggingUtil.getLogFile(Misc.formatInvariant(
                "DriveRampRegression-%d.csv", System.currentTimeMillis())));

        telemetry.clearAll();
        telemetry.addLine("Quasi-static ramp up test complete");
        if (fitIntercept) {
            telemetry.addLine(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
                    rampResult.kV, rampResult.kStatic, rampResult.rSquare));
        } else {
            telemetry.addLine(Misc.formatInvariant("kV = %.5f (R^2 = %.2f)",
                    rampResult.kStatic, rampResult.rSquare));
        }
        telemetry.addLine("Would you like to fit kA?");
        telemetry.addLine("Press (A) for yes, (B) for no");
        telemetry.update();



        boolean fitAccelFF = false;
        while (!isStopRequested()) {
            if (gamepad1.a) {
                fitAccelFF = true;
                while (!isStopRequested() && gamepad1.a) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (!isStopRequested() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        if (fitAccelFF) {
            telemetry.clearAll();
            telemetry.addLine("Place the robot back in its starting position");
            telemetry.addLine("Press (A) to continue");
            telemetry.update();

            while (!isStopRequested() && !gamepad1.a) {
                idle();
            }
            while (!isStopRequested() && gamepad1.a) {
                idle();
            }

            telemetry.clearAll();
            telemetry.addLine("Running...");
            telemetry.update();


            double maxPowerTime = DISTANCE / maxVel;

            startTime = clock.seconds();
            AccelRegression accelRegression = new AccelRegression();

            drive.setPoseEstimate(new Pose2d());
            drive.setDrivePower(new Pose2d(MAX_POWER, 0.0, 0.0));
            while (!isStopRequested()) {
                double elapsedTime = clock.seconds() - startTime;
                if (elapsedTime > maxPowerTime) {
                    break;
                }


                accelRegression.add(elapsedTime, drive.getPoseEstimate().getX(), MAX_POWER);

                drive.updatePoseEstimate();

            }
            drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

            AccelRegression.AccelResult accelResult = accelRegression.fit(
                    rampResult.kV, rampResult.kStatic);

            accelRegression.save(LoggingUtil.getLogFile(Misc.formatInvariant(
                    "DriveAccelRegression-%d.csv", System.currentTimeMillis())));

            telemetry.clearAll();
            telemetry.addLine("Constant power test complete");
            telemetry.addLine(Misc.formatInvariant("kA = %.5f (R^2 = %.2f)",
                    accelResult.kA, accelResult.rSquare));
            telemetry.update();
        }

        while (!isStopRequested()) {
            idle();
        }
    }

}
