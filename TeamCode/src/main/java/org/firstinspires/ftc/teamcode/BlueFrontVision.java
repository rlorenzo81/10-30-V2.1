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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Blue front vision", group="Pushbot")
@Disabled
public class BlueFrontVision extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private static final String VUFORIA_KEY =
            " ARP0mN//////AAABmXcMqq49h0n9vE1XHbK47eUX9Cct/dieBsWPAdQMUfrGC+q7wz6tprle0Bzy9DOblmDGyD3gsZOD8jMLWX7jGeXhtPfLVzvZQXcSbuf6cAe/Hmf1z1EdDJKHddZYUUBKdMDMSL/1w4OcXNxRkm3qENngGQhw/UnX/g+w/8GD4w7VmsXKaPA1mrYkeqvvk/eR3CvmTr4Q2qBcD053usBqe502+oHQr9IQHZbDnC7a728o5Fy/+3GIPYrJX9E05l3e/YSpcKoRRDBAPcmp7hLgYgPS0nuj3yT7UyeR5zzlL+ps4mM4Wvyl4P4dftoI00UMOzUOxynEkdUtlR7sbx9aTAgryTyDOsryTlZ5jVmfO29E";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    private ElapsedTime     runtime = new ElapsedTime();
    public boolean isDuckDetected = false;
    public double right;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    // private ElapsedTime     runtime = new ElapsedTime();

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    static final double COUNTS_PER_MOTOR_REV = 493;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.25;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 3;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable starts at 0.1
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable starts at 0.15

    static final double     FORWARD_SPEED = 0.6;


    Orientation             lastAngles = new Orientation();
    double                  globalAngle;



    @Override
    public void runOpMode() {

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //byte AXIS_MAP_CONFIG_BYTE = 0b000110; // to swap x and z axes
        byte AXIS_MAP_CONFIG_BYTE = 0b011000; //to swap y and z

        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

//Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        sleep(100); //Changing modes requires a delay before doing anything else

//Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0b111111
        );

//Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0b111);

//Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0b1111);

        sleep(100); //Changing modes again requires a delay
        // Set up our telemetry dashboard
        composeTelemetry();

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        imu.initialize(parameters);

        runtime.reset();
        while (opModeIsActive()&& runtime.seconds()<5){
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    //  ** ADDED **
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;

                        // check label to see if the camera now sees a Duck         ** ADDED **
                        if (recognition.getLabel().equals("Duck")) {            //  ** ADDED **
                            isDuckDetected = true;//  ** ADDED **
                            right=recognition.getLeft();
                            telemetry.addData("Object Detected", "Duck");      //  ** ADDED **
                        } else {                                               //  ** ADDED **
                            isDuckDetected = false;                            //  ** ADDED **
                        }                                                      //  ** ADDED **
                    }
                    telemetry.update();
                }
            }
        }

        if(isDuckDetected && right <= 223){

            strafeRight(0.5,0.5,0.5,0.5,52,0);

            robot.rightLift.setPower(-0.5);

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);
            sleep(2000);

            robot.rightLift.setPower(0);

            gyroDrive(0.5,0.5,0.5,0.5,6,0);

            robot.leftArm.setPower(0.6);

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);
            sleep(2000);

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);

            robot.leftArm.setPower(0);

            gyroReverse(0.5,0.5,0.5,0.5,28,0);

            strafeLeft(0.5,0.5,0.5,0.5,46,0);

            robot.rightDrive.setPower(-0.6);

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);
            sleep(3000);

            strafeRight(0.5,0.5,0.5,0.5,22,0);

        }

        else if (isDuckDetected && right >=223){
            gyroReverse(0.5,0.5,0.5,0.5,2,0);

            strafeRight(0.5,0.5,0.5,0.5,52,0);

            gyroDrive(0.5,0.5,0.5,0.5,7,0);


            robot.rightLift.setPower(-0.5);


            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);
            sleep(5500);

            robot.rightLift.setPower(0);

            robot.leftArm.setPower(0.6);

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);
            sleep(2000);

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);

            robot.leftArm.setPower(0);

            gyroReverse(0.5,0.5,0.5,0.5,28,0);

            strafeLeft(0.5,0.5,0.5,0.5,46,0);

            robot.rightDrive.setPower(-0.6);

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);
            sleep(3000);

            strafeRight(0.5,0.5,0.5,0.5,24,0);

            gyroReverse(0.5,0.5,0.5,0.5,2,0);
        }

        else if (!isDuckDetected){

            gyroReverse(0.5,0.5,0.5,0.5,2,0);

            strafeRight(0.5,0.5,0.5,0.5,52,0);

            gyroDrive(0.5,0.5,0.5,0.5,5,0);


            robot.rightLift.setPower(-0.6);


            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);
            sleep(5000);

            robot.rightLift.setPower(0);

            robot.leftArm.setPower(0.6);

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);
            sleep(2000);

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);

            robot.leftArm.setPower(0);

            gyroReverse(0.5,0.5,0.5,0.5,28,0);
            strafeLeft(0.5,0.5,0.5,0.5,46,0);

            robot.rightDrive.setPower(-0.6);

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);
            sleep(3000);

            strafeRight(0.5,0.5,0.5,0.5,22,0);
            gyroReverse(0.5,0.5,0.5,0.5,3,0);
        }





    }
    public void strafeLeft ( double speedLF,double speedRF, double speedLR, double speedRR,
                             double distance,
                             double angles) {

        int newLeftTargetF;
        int newLeftTargetR;
        int newRightTargetF;
        int newRightTargetR;
        int moveCounts;
        double max;
        double max2;
        double error;
        double steer;
        double leftSpeedF;
        double leftSpeedR;
        double rightSpeedF;
        double rightSpeedR;

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            //newLeftTargetF = robot.leftFront.getCurrentPosition() + moveCounts;
            //newRightTargetF = robot.rightFront.getCurrentPosition() + moveCounts;
            newLeftTargetR = robot.rightFront.getCurrentPosition() + moveCounts; //was leftrear
            //   newRightTargetR = robot.rightRear.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            //robot.leftFront.setTargetPosition(newLeftTargetF);
            // robot.rightFront.setTargetPosition(newRightTargetF);
            //robot.leftRear.setTargetPosition(newLeftTargetR);
            //robot.rightRear.setTargetPosition(newRightTargetR);

            robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // start motion.
            speedLF = Range.clip(Math.abs(speedLF), -1, 1.0);
            speedRF = Range.clip(Math.abs(speedRF), -1, 1.0);
            speedLR = Range.clip(Math.abs(speedLR), -1, 1.0);
            speedRR = Range.clip(Math.abs(speedRR), -1, 1.0);

            robot.leftFront.setPower(-speedLF);
            robot.rightFront.setPower(speedRF);
            robot.leftRear.setPower(speedLR);
            robot.rightRear.setPower(-speedRR);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    robot.rightFront.getCurrentPosition() < newLeftTargetR) {

                // adjust relative speed based on heading error.
                error = getError(angles);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeedF = -speedLF - (steer * .1);
                leftSpeedR = speedLR - (steer * .1);
                rightSpeedF = speedRF + (steer * .1);
                rightSpeedR = -speedRR + (steer * .1);

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeedF), Math.abs(rightSpeedR));
                max2 = Math.max(Math.abs(leftSpeedR), Math.abs(rightSpeedF));
                if (max > 1.0 || max2 > 1) {
                    leftSpeedF /= max;
                    leftSpeedR /= max;
                    rightSpeedF /= max;
                    rightSpeedR /= max;

                }


                robot.leftFront.setPower(leftSpeedF);
                robot.rightFront.setPower(rightSpeedF);
                robot.leftRear.setPower(leftSpeedR);
                robot.rightRear.setPower(rightSpeedR);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d", newLeftTargetR);
                telemetry.addData("Actual", "%7d:%7d", robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeedF, rightSpeedR);
                telemetry.update();
            }


            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightFront.setPower(0);
            // Turn off RUN_TO_POSITION
            /*robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            */




          /*  robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
*/

        }
    }

    public void strafeRight ( double speedLF,double speedRF, double speedLR, double speedRR,
                              double distance,
                              double angles) {

        int newLeftTargetF;
        int newLeftTargetR;
        int newRightTargetF;
        int newRightTargetR;
        int moveCounts;
        double max;
        double max2;
        double error;
        double steer;
        double leftSpeedF;
        double leftSpeedR;
        double rightSpeedF;
        double rightSpeedR;

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            //newLeftTargetF = robot.leftFront.getCurrentPosition() + moveCounts;
            //newRightTargetF = robot.rightFront.getCurrentPosition() + moveCounts;
            newLeftTargetR = robot.rightRear.getCurrentPosition() + moveCounts; //was leftrear
            //   newRightTargetR = robot.rightRear.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            //robot.leftFront.setTargetPosition(newLeftTargetF);
            // robot.rightFront.setTargetPosition(newRightTargetF);
            //robot.leftRear.setTargetPosition(newLeftTargetR);
            //robot.rightRear.setTargetPosition(newRightTargetR);

            robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // start motion.
            speedLF = Range.clip(Math.abs(speedLF), -1, 1.0);
            speedRF = Range.clip(Math.abs(speedRF), -1, 1.0);
            speedLR = Range.clip(Math.abs(speedLR), -1, 1.0);
            speedRR = Range.clip(Math.abs(speedRR), -1, 1.0);

            robot.leftFront.setPower(speedLF);
            robot.rightFront.setPower(-speedRF);
            robot.leftRear.setPower(-speedLR);
            robot.rightRear.setPower(speedRR);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    robot.rightRear.getCurrentPosition() < newLeftTargetR) {

                // adjust relative speed based on heading error.
                error = getError(angles);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeedF = speedLF - (steer * .2);
                leftSpeedR = -speedLR -(steer * .2);
                rightSpeedF = -speedRF + (steer * .2);
                rightSpeedR = speedRR + (steer * .2);

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeedF), Math.abs(rightSpeedR));
                max2 = Math.max(Math.abs(leftSpeedR), Math.abs(rightSpeedF));
                if (max > 1.0 || max2 > 1) {
                    leftSpeedF /= max;
                    leftSpeedR /= max;
                    rightSpeedF /= max;
                    rightSpeedR /= max;

                }


                robot.leftFront.setPower(leftSpeedF);
                robot.rightFront.setPower(rightSpeedF);
                robot.leftRear.setPower(leftSpeedR);
                robot.rightRear.setPower(rightSpeedR);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d", newLeftTargetR);
                telemetry.addData("Actual", "%7d:%7d", robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeedF, rightSpeedR);
                telemetry.update();
            }


            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightFront.setPower(0);
            // Turn off RUN_TO_POSITION
            /*robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            */




          /*  robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
*/

        }
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     *
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angles      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speedLF,double speedRF, double speedLR, double speedRR,
                            double distance,
                            double angles) {

        int     newLeftTargetF;
        int     newLeftTargetR;
        int     newRightTargetF;
        int     newRightTargetR;
        int     moveCounts;
        double  max;
        double max2;
        double  error;
        double  steer;
        double  leftSpeedF;
        double  leftSpeedR;
        double  rightSpeedF;
        double  rightSpeedR;

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // Ensure that the opmode is still active
        if (opModeIsActive()) {
           /* robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
*/
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTargetF = robot.rightFront.getCurrentPosition() + moveCounts;
            // newRightTargetF = robot.rightFront.getCurrentPosition() + moveCounts;
            //  newLeftTargetR = robot.leftRear.getCurrentPosition() + moveCounts;
            //   newRightTargetR = robot.rightRear.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            //robot.leftFront.setTargetPosition(newLeftTargetF);
            // robot.rightFront.setTargetPosition(newRightTargetF);
            //robot.leftRear.setTargetPosition(newLeftTargetR);
            //robot.rightRear.setTargetPosition(newRightTargetR);

            robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // start motion.
            speedLF = Range.clip(Math.abs(speedLF), -1, 1.0);
            speedRF = Range.clip(Math.abs(speedRF), -1, 1.0);
            speedLR = Range.clip(Math.abs(speedLR), -1, 1.0);
            speedRR = Range.clip(Math.abs(speedRR), -1, 1.0);

            robot.leftFront.setPower(speedLF);
            robot.rightFront.setPower(speedRF);
            robot.leftRear.setPower(speedLR);
            robot.rightRear.setPower(speedRR);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    robot.leftFront.getCurrentPosition() < newLeftTargetF ) {

                // adjust relative speed based on heading error.
                error = getError(angles);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeedF = speedLF - (steer*.05);
                leftSpeedR = speedLR - (steer*.05);
                rightSpeedF = speedRF + (steer*.05);
                rightSpeedR= speedRR + (steer*.05);

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeedF), Math.abs(rightSpeedR));
                max2 = Math.max(Math.abs(leftSpeedR), Math.abs(rightSpeedF));
                if (max > 1.0 || max2 >1)
                {
                    leftSpeedF /= max;
                    leftSpeedR /= max;
                    rightSpeedF /= max;
                    rightSpeedR /= max;

                }



                robot.leftFront.setPower(leftSpeedF);
                robot.rightFront.setPower(rightSpeedF);
                robot.leftRear.setPower(leftSpeedR);
                robot.rightRear.setPower(rightSpeedR);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d",      newLeftTargetF);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeedF, rightSpeedR);
                telemetry.update();
            }



            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightFront.setPower(0);
            // Turn off RUN_TO_POSITION
            /*robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            */





        }
    }

    public void robotStop(double zero, long timeToWait ){
        robot.leftFront.setPower(zero);
        robot.rightFront.setPower(zero);
        robot.leftRear.setPower(zero);
        robot.rightFront.setPower(zero);

        sleep(timeToWait);
    }








    public void gyroReverse ( double speedLF,double speedRF, double speedLR, double speedRR,
                              double distance,
                              double angles) {

        int     newLeftTargetF;
        int     newLeftTargetR;
        int     newRightTargetF;
        int     newRightTargetR;
        int     moveCounts;
        double  max;
        double max2;
        double  error;
        double  steer;
        double  leftSpeedF;
        double  leftSpeedR;
        double  rightSpeedF;
        double  rightSpeedR;

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTargetF = robot.leftFront.getCurrentPosition() - moveCounts;
            // newRightTargetF = robot.rightFront.getCurrentPosition() + moveCounts;
            //  newLeftTargetR = robot.leftRear.getCurrentPosition() + moveCounts;
            //   newRightTargetR = robot.rightRear.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            //robot.leftFront.setTargetPosition(newLeftTargetF);
            // robot.rightFront.setTargetPosition(newRightTargetF);
            //robot.leftRear.setTargetPosition(newLeftTargetR);
            //robot.rightRear.setTargetPosition(newRightTargetR);

            robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // start motion.
            speedLF = Range.clip(Math.abs(speedLF), -1, 1.0);
            speedRF = Range.clip(Math.abs(speedRF), -1, 1.0);
            speedLR = Range.clip(Math.abs(speedLR), -1, 1.0);
            speedRR = Range.clip(Math.abs(speedRR), -1, 1.0);

            robot.leftFront.setPower(-speedLF);
            robot.rightFront.setPower(-speedRF);
            robot.leftRear.setPower(-speedLR);
            robot.rightRear.setPower(-speedRR);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    robot.leftFront.getCurrentPosition() > newLeftTargetF ) {

                // adjust relative speed based on heading error.
                error = getError(angles);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeedF = -speedLF - (steer*.2);
                leftSpeedR = -speedLR - (steer*.2);
                rightSpeedF = -speedRF + (steer*.2);
                rightSpeedR= -speedRR +(steer*.2);

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeedF), Math.abs(rightSpeedR));
                max2 = Math.max(Math.abs(leftSpeedR), Math.abs(rightSpeedF));
                if (max > 1.0 || max2 >1)
                {
                    leftSpeedF /= max;
                    leftSpeedR /= max;
                    rightSpeedF /= max;
                    rightSpeedR /= max;

                }



                robot.leftFront.setPower(leftSpeedF);
                robot.rightFront.setPower(rightSpeedF);
                robot.leftRear.setPower(leftSpeedR);
                robot.rightRear.setPower(rightSpeedR);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d",      newLeftTargetF);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeedF, rightSpeedR);
                telemetry.update();
            }



            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightFront.setPower(0);
            // Turn off RUN_TO_POSITION





        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angles)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angles      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angles) {

        //try the following:



        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angles, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();

            /*robot.leftFront.setPower(speed);
            robot.rightFront.setPower(-speed);
            robot.leftRear.setPower(speed);
            robot.rightRear.setPower(-speed);*/
        }


    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angles      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angles, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angles, P_TURN_COEFF);
            telemetry.update();
        }


        // Stop all motion;
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);

    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angles     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angles, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angles);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer*0.2;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFront.setPower(leftSpeed);
        robot.rightFront.setPower(rightSpeed);
        robot.leftRear.setPower(leftSpeed);
        robot.rightRear.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angles);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;


        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;


    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */


    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

}