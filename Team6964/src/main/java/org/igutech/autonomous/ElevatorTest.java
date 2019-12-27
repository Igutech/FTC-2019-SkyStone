package org.igutech.autonomous;

        import com.acmerobotics.roadrunner.util.NanoClock;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.igutech.autonomous.roadrunner.Elevator;

/*
 * Simple test of motion-profiled elevator autonomous operation. The elevator should move *smoothly*
 * between random heights.
 */
@Autonomous(group = "elevator")
public class ElevatorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Elevator elevator = new Elevator(hardwareMap);
        NanoClock clock = NanoClock.system();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            elevator.setHeight(10.0);

            double startTime = clock.seconds();
            while (!isStopRequested()) {
                elevator.update();
            }
        }
    }
}