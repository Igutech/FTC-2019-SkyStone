package org.igutech.autonomous;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.igutech.autonomous.roadrunner.Elevator;

/*
 * Simple test of motion-profiled elevator autonomous operation. The elevator should move *smoothly*
 * between random heights.
 */
@Disabled
@Autonomous(group = "elevator")
public class ElevatorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Elevator elevator = new Elevator(hardwareMap);
        NanoClock clock = NanoClock.system();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            elevator.setHeight(Elevator.MAX_HEIGHT * Math.random());

            double startTime = clock.seconds();
            while (!isStopRequested() && (clock.seconds() - startTime) < 5) {
                elevator.update();
            }
        }
    }
}