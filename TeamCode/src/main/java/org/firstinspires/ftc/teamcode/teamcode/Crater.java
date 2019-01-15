package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.OpMode8696;

/**
 * Created by USER on 2/24/2018.
 */

/* From lander to drop-zone 4.5
From drop zone to crater 8.5*/
@Autonomous(name = "Crater", group = "test")
public class Crater extends OpMode8696 {

    @Override
    public void runOpMode(){

        initRobot();

        waitForStart();

        lift.setPower(1);
        sleep(3400);

        lift.setPower(0);
        sleep(1000);

        latch.setPower(1);
        sleep(8000);

        latch.setPower(0);
        sleep(1000);

        leftFront.setPower(-1);
        leftBack.setPower(-1);
        rightFront.setPower(1);
        rightBack.setPower(1);

        sleep(600);
    }

}
