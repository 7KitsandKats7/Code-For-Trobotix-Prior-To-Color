package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.OpMode8696;

/**
 * Created by USER on 2/24/2018.
 */

/* From lander to drop-zone 4.5
From drop zone to crater 8.5*/
@Autonomous(name = "Red Depo", group = "test")
public class RedDepo extends OpMode8696 {

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

        leftFront.setPower(0.6);
        rightFront.setPower(0.6);

        sleep(2500);

        leftFront.setPower(0);
        rightFront.setPower(0);

        sleep(1000);

        flipper.setPosition(90);

        sleep(1000);

        flipper.setPosition(0);

        leftFront.setPower(-0.2);
        rightFront.setPower(-0.2);

        sleep(1000);

        leftFront.setPower(0);
        rightFront.setPower(0);

        leftFront.setPower(1);
        rightFront.setPower(-1);

        sleep(1500);

        leftFront.setPower(0);
        rightFront.setPower(0);

        sleep(1000);

        rightFront.setPower(0.8);
        leftFront.setPower(0.6);

        sleep(3700);

        rightFront.setPower(0);
        leftFront.setPower(0);






    }

}
