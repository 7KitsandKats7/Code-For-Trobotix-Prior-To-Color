package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.OpMode8696;

/**
 * Created by USER on 2/24/2018.
 */

/* From lander to drop-zone 4.5
From drop zone to crater 8.5*/
@Autonomous(name = "FullDepo", group = "test")
public class FullDepo extends OpMode8696 {

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

        leftFront.setPower(-0.7);
        leftBack.setPower(-0.7);
        rightFront.setPower(0.7);
        rightBack.setPower(0.7);

        sleep(1300);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        leftPivot.setPower(1);
        rightPivot.setPower(1);

        sleep(500);

        leftPivot.setPower(0);
        rightPivot.setPower(0);

        sleep(1000);

        leftFront.setPower(-0.6);
        leftBack.setPower(-0.6);
        rightFront.setPower(-0.6);
        rightBack.setPower(-0.6);

        sleep(500);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        sleep(500);

        leftBack.setPower(1);
        leftFront.setPower(-1);
        rightBack.setPower(1);
        rightFront.setPower(-1);

        sleep(3200);

    }

}
