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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.PwmControl;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop Mechanum", group="Pushbot")
//@Disabled
public class MechanumTeleop extends OpMode{

    /* Declare OpMode members. */
   HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");

        //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


        robot.rightClaw.setPosition(0.4);
        robot.leftClaw.setPosition(0.6);


    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    @Override
    public void loop() {

        // Set the rotation servo for extended PWM range
       /* if (robot.rotateServo.getController() instanceof ServoControllerEx) {
// Confirm its an extended range servo controller before we try to set to avoid crash
            ServoControllerEx theControl = (ServoControllerEx) robot.rotateServo.getController();
            int thePort = robot.rotateServo.getPortNumber();
            PwmControl.PwmRange theRange = new PwmControl.PwmRange(400, 2700);
            theControl.setServoPwmRange(thePort, theRange);
        }*/
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.sin(robotAngle) -rightX;
        final double v2 = r * Math.cos(robotAngle) +rightX;
        final double v3 = r * Math.cos(robotAngle) - rightX;
        final double v4 = r * Math.sin(robotAngle) +rightX;

        robot.leftFront.setPower(-v1);
        robot.rightFront.setPower(-v2);
        robot.leftRear.setPower(-v3);
        robot.rightRear.setPower(-v4);

        if (gamepad1.dpad_left){
            robot.leftFront.setPower(-0.4);
            robot.leftRear.setPower(0.4);
            robot.rightFront.setPower(0.4);
            robot.rightRear.setPower(-0.4);
        }

        else if (gamepad1.dpad_right){
            robot.leftFront.setPower(0.4);
            robot.leftRear.setPower(-0.4);
            robot.rightFront.setPower(-0.4);
            robot.rightRear.setPower(0.4);
        }



        double left;
        double right;

        double left1;
        double right1;

        double lift;
        double intake;

        lift = gamepad2.right_stick_y;
        if(lift >= 0.3 ||lift <=-.3) {
            robot.rightLift.setPower(-lift);
        }
        else {
            robot.rightLift.setPower(0);
        }

if(gamepad2.x){
    robot.leftArm.setPower(1); //intake
}

else if(gamepad2.b){
    robot.leftArm.setPower(-1);
}

else if(gamepad2.a){
    robot.leftArm.setPower(0);
}

if(gamepad2.left_bumper){
    robot.rightDrive.setPower(1);
}
else if(gamepad2.right_bumper){
    robot.rightDrive.setPower(0);
        }
/*
        intake = gamepad2.left_stick_y;
        robot.leftArm.setPower(intake);
*/






        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad2.left_stick_y;
        right = gamepad2.left_stick_y;

       // left1 =-gamepad2.right_stick_y;
        //right1 =gamepad2.right_stick_y;


        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);


/*
        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad2.x) {
            robot.blockFlip.setPosition(0.8);
        }
        else if (gamepad2.b){
            robot.blockFlip.setPosition(0.1);
        }
        if (gamepad2.y) {
            robot.intakeFlip.setPosition(0.1);
        }
        else if (gamepad2.a){
            robot.intakeFlip.setPosition(0.7);
        }
*/
        if(gamepad2.left_bumper){
            robot.rightClaw.setPosition(0.4);
            robot.leftClaw.setPosition(0.6);
        }//left bumper opens claw

        else if (gamepad2.right_bumper){
            robot.rightClaw.setPosition(0.1);
            robot.leftClaw.setPosition(0.9);

        } //right bumper closes claw


      /*  if(gamepad1.left_bumper){
            robot.rightFoundation.setPosition(1);
            robot.leftFoundation.setPosition(0);

        }

        else if(gamepad1.right_bumper){

            robot.rightFoundation.setPosition(0);
            robot.leftFoundation.setPosition(1);

        }

       */
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}