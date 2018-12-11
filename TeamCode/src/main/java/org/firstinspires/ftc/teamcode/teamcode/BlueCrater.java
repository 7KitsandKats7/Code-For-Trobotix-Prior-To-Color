package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.OpMode8696;

/**
 * Created by USER on 2/24/2018.
 */

/* From lander to drop-zone 4.5
From drop zone to crater 8.5*/
@Autonomous(name = "Blue Crater", group = "test")
public class BlueCrater extends OpMode8696 {

    @Override
    public void runOpMode(){

        initRobot();
        initGyro();

        waitForStart();

        lift.setPower(0.5);
        sleep(3400);

        lift.setPower(0);
        sleep(1000);

        latch.setPower(-0.5);
        sleep(3100);

        latch.setPower(0);
        sleep(1000);

        leftFront.setPower(1);
       rightFront.setPower(1);

       sleep(1350);

    }

}
