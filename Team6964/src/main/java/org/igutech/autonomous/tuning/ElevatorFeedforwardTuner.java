package org.igutech.autonomous.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.tuning.AccelRegression;
import com.acmerobotics.roadrunner.tuning.RampRegression;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.igutech.autonomous.roadrunner.Elevator;
import org.igutech.autonomous.tuning.LoggingUtil;

/*
 * Op mode for computing kV, kStatic, and kA from various elevator motions. Note: for those using the
 * built-in PID, **kStatic and kA should not be tuned**. For the curious, here's an outline of the
 * basic procedure:
 *   1. Slowly ramp the motor power and record encoder values along the way.
 *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 *      and an optional intercept (kStatic).
 *   3. Accelerate the elevator (apply constant power) and record the encoder counts.
 *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
 *      regression.
 */
@Disabled
@Config
@Autonomous(group = "elevator")
public class ElevatorFeedforwardTuner extends LinearOpMode {
    public static final double MAX_POWER = 0.7;
    public static final double DISTANCE = 0.75 * Elevator.MAX_HEIGHT;

    @Override
    public void runOpMode() throws InterruptedException {
        Elevator elevator = new Elevator(hardwareMap);

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

        telemetry.log().clear();
        telemetry.log().add("Press (A) to begin");
        telemetry.update();

        while (!isStopRequested() && !gamepad1.a) {
            idle();
        }
        while (!isStopRequested() && gamepad1.a) {
            idle();
        }

        telemetry.log().clear();
        telemetry.log().add("Running...");
        telemetry.update();

        double maxVel = Elevator.rpmToVelocity(Elevator.getMaxRpm());
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);

        double startTime = clock.seconds();
        RampRegression rampRegression = new RampRegression();

        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            rampRegression.add(elapsedTime, elevator.getCurrentHeight(), power);

            elevator.setPower(power);
        }
        elevator.setPower(0);

        RampRegression.RampResult rampResult = rampRegression.fit(fitIntercept);

        rampRegression.save(LoggingUtil.getLogFile(Misc.formatInvariant(
                "ElevatorRampRegression-%d.csv", System.currentTimeMillis())));

        telemetry.log().clear();
        telemetry.log().add("Quasi-static ramp up test complete");
        if (fitIntercept) {
            telemetry.log().add(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
                    rampResult.kV, rampResult.kStatic, rampResult.rSquare));
        } else {
            telemetry.log().add(Misc.formatInvariant("kV = %.5f (R^2 = %.2f)",
                    rampResult.kV, rampResult.rSquare));
        }
        telemetry.log().add("Would you like to fit kA?");
        telemetry.log().add("Press (A) for yes, (B) for no");
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
            telemetry.log().clear();
            telemetry.log().add("Press (A) to continue");
            telemetry.update();

            while (!isStopRequested() && !gamepad1.a) {
                idle();
            }
            while (!isStopRequested() && gamepad1.a) {
                idle();
            }

            telemetry.log().clear();
            telemetry.log().add("Running...");
            telemetry.update();

            double maxPowerTime = 0.75 * DISTANCE / maxVel; // 0.75 = "safety factor"

            startTime = clock.seconds();
            AccelRegression accelRegression = new AccelRegression();

            elevator.setPower(-MAX_POWER);
            while (!isStopRequested()) {
                double elapsedTime = clock.seconds() - startTime;
                if (elapsedTime > maxPowerTime) {
                    break;
                }

                accelRegression.add(elapsedTime, elevator.getCurrentHeight(), -MAX_POWER);
            }
            elevator.setPower(0);

            AccelRegression.AccelResult accelResult = accelRegression.fit(
                    rampResult.kV, rampResult.kStatic);

            accelRegression.save(LoggingUtil.getLogFile(Misc.formatInvariant(
                    "ElevatorAccelRegression-%d.csv", System.currentTimeMillis())));

            telemetry.log().clear();
            telemetry.log().add("Constant power test complete");
            telemetry.log().add(Misc.formatInvariant("kA = %.5f (R^2 = %.2f)",
                    accelResult.kA, accelResult.rSquare));
            telemetry.update();
        }

        while (!isStopRequested()) {
            idle();
        }
    }
}