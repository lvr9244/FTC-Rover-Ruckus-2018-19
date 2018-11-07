/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


@Autonomous(name="Auto Red Audience Encoders", group ="Concept")
    //@Disabled
    public class Auto_Red_Audience_Encoders extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 1680;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // or figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AfFRRyH/////AAABmb6QHOtIZE6TrGJe24Gin0kkndTn8W4iKeV93VNh/+DEMbRfjix210MNGCcZqvvT62/ILpMXNQUJghRt2EP9hq27uBdJfnP2EtiRy1AZEqQqar3l/QVLkxFZFKNfkt7SjqUjyuOfFKa/bmlhXLmpo3WEO+tDVwxUW20kxvqgkryeap0FkIhNTb3uCuuGZCF0x2FO8J1Dm5Scyf8BdM6hrJfcdMemZ8p5v6T155q1EWW3IAqI6QZR3V8Bd+wjbzAA/mHIR/IFdGxtDzSHtwV2lecT/gagIvrjrsQ5cDaickzpv+Y2Wv+vARwpoQrUqf11AS0Fqdg3hqDC90tOtX6B5npiHp3B9pahqV/kxMZeCV/u";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    private DcMotor leftMotorFront = null;
    private DcMotor rightMotorFront = null;
    private DcMotor rightMotorRear = null;
    private DcMotor leftMotorRear = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        rightMotorRear = hardwareMap.dcMotor.get("rightMotorRear");
        leftMotorRear = hardwareMap.dcMotor.get("leftMotorRear");


        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery

        leftMotorFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotorRear.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotorFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotorRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        telemetry.update();
        waitForStart();




        }


    public void motorOff() {


        leftMotorFront.setPower(0);

        rightMotorFront.setPower(0);

        leftMotorRear.setPower(0);

        rightMotorRear.setPower(0);

    }
    public void driveForward(double speed) {

        leftMotorFront.setPower(speed);

        rightMotorFront.setPower(speed);

        leftMotorRear.setPower(speed);

        rightMotorRear.setPower(speed);

    }
    public void driveLeft(double speed) {

        leftMotorFront.setPower(-speed);

        rightMotorFront.setPower(speed);

        leftMotorRear.setPower(speed);

        rightMotorRear.setPower(-speed);

    }
    public void driveRight(double speed) {

        leftMotorFront.setPower(speed);

        rightMotorFront.setPower(-speed);

        leftMotorRear.setPower(-speed);

        rightMotorRear.setPower(speed);

    }
    public void driveBackward(double speed) {

        leftMotorFront.setPower(-speed);

        rightMotorFront.setPower(-speed);

        leftMotorRear.setPower(-speed);

        rightMotorRear.setPower(-speed);

    }
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches, double leftRearInches, double rightRearInches,
                             double timeoutS) {

        int newLeftFrontTarget;

        int newRightFrontTarget;

        int newLeftRearTarget;

        int newRightRearTarget;


        // Ensure that the opmode is still active

        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller

            newLeftFrontTarget = leftMotorFront.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);

            newLeftRearTarget = leftMotorRear.getCurrentPosition() + (int) (leftRearInches * COUNTS_PER_INCH);

            newRightFrontTarget = rightMotorFront.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);

            newRightRearTarget = rightMotorRear.getCurrentPosition() + (int) (rightRearInches * COUNTS_PER_INCH);

            leftMotorFront.setTargetPosition(newLeftFrontTarget);

            rightMotorFront.setTargetPosition(newRightFrontTarget);

            leftMotorRear.setTargetPosition(newLeftRearTarget);

            rightMotorRear.setTargetPosition(newRightRearTarget);


            // Turn On RUN_TO_POSITION

            leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftMotorRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightMotorRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.

            runtime.reset();
            leftMotorFront.setPower(Math.abs(speed));

            rightMotorFront.setPower(Math.abs(speed));

            leftMotorRear.setPower(Math.abs(speed));

            rightMotorRear.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.

            while (opModeIsActive() &&

                    (runtime.seconds() < timeoutS) &&

                    (leftMotorFront.isBusy() && rightMotorFront.isBusy() && leftMotorRear.isBusy() && rightMotorRear.isBusy())) {


                // Display it for the driver.

                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftRearTarget, newRightRearTarget);

                telemetry.addData("Path2", "Running at %7d :%7d",

                        leftMotorFront.getCurrentPosition(),

                        rightMotorFront.getCurrentPosition(),

                        leftMotorRear.getCurrentPosition(),

                        rightMotorRear.getCurrentPosition());

                telemetry.update();

            }


            // Stop all motion;

            leftMotorFront.setPower(0);

            rightMotorFront.setPower(0);

            leftMotorRear.setPower(0);

            rightMotorRear.setPower(0);


            // Turn off RUN_TO_POSITION

            leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move

        }
    }

}

