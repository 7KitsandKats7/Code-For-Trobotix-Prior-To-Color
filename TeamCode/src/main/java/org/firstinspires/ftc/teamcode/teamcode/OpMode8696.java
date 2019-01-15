package org.firstinspires.ftc.teamcode.teamcode;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TroTeleOP", group="Linear Opmode")
public class TroTeleOp extends OpMode8696 {

    //No variables need to be initialized here. Yay!

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initializes the robots parts, including wheels and lifting mechanism.
        initRobot();
        waitForStart();
        
        //Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Sets power for the tank drive mecanum wheels. We use the left and right gamepad 1 sticks in order to do this.
            leftBack.setPower(gamepad1.left_stick_y);
            leftFront.setPower(gamepad1.left_stick_y);
            rightBack.setPower(-gamepad1.right_stick_y);
            rightFront.setPower(-gamepad1.right_stick_y);

            //Strafing right for the right bumper and left for the left bumper.
            //Helps with aligning our robot to the lander correctly.
            if (gamepad1.right_bumper){
                leftBack.setPower(-1);
                leftFront.setPower(1);
                rightBack.setPower(-1);
                rightFront.setPower(1);
            }
            if (gamepad1.left_bumper){
                leftBack.setPower(1);
                leftFront.setPower(-1);
                rightBack.setPower(1);
                rightFront.setPower(-1);
            }

            //All other controls besides movement reside on the second controller. It makes the driver's life easier.

            //Lift controls are set on the Dpad. Originally we had the latch also on the left and right
            //dpad, but the controls would interfere.
            if (gamepad2.dpad_down) lift.setPower(-1);
            if (gamepad2.dpad_up) lift.setPower(1);
            if (!gamepad2.dpad_down && !gamepad2.dpad_up) lift.setPower(0);

            //Latch controls put on x and b for easier reach.
            if (gamepad2.x) latch.setPower(-1);
            if (gamepad2.b) latch.setPower(1);
            if (!gamepad2.x && !gamepad2.b) latch.setPower(0);

            //This pivots the sliders to whatever position we desire.
            //We have to put a lower speed on going down to combat gravity.
            if (-gamepad2.left_stick_y > 0){
                leftPivot.setPower(0.3);
                rightPivot.setPower(0.3);
            }
            if (-gamepad2.left_stick_y < 0){
                leftPivot.setPower(-0.6);
                rightPivot.setPower(-0.6);
            }
            if (-gamepad2.left_stick_y == 0){
                leftPivot.setPower(0);
                rightPivot.setPower(0);
            }

            //This pivots the sliders to whatever position we desire.
            //We have to put a lower speed on going down to combat gravity.
            if (-gamepad2.right_stick_y > 0){
                leftPivot.setPower(0.5);
                rightPivot.setPower(0.5);
            }
            if (-gamepad2.right_stick_y < 0){
                leftPivot.setPower(-1);
                rightPivot.setPower(-1);
            }
            if (-gamepad2.right_stick_y == 0){
                leftPivot.setPower(0);
                rightPivot.setPower(0);
            }

            //Big Bertha is what our engineers called the motor that slides
            //the slider in and out. We kept it like that in code.
            if (gamepad2.a){
                bigBertha.setPower(-1);
            }
            if (gamepad2.y){
                bigBertha.setPower(1);
            }
            if (!gamepad2.y && !gamepad2.a){
                bigBertha.setPower(0);
            }

            // Perfect latch timing in order to prepare the robot for the auto.
            if (gamepad1.x){
                latch.setPower(-1);
                sleep(6000);
                latch.setPower(0);
            }

        }
    }
}
