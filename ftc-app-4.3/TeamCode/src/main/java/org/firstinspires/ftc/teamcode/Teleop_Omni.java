/*

Copyright (c) 2016 Robert Atkinson



All rights reserved.



Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:



Redistributions of source code must retain the above copyright notice, this list

of conditions and the following disclaimer.



Redistributions in binary form must reproduce the above copyright notice, this

list of conditions and the following disclaimer in the documentation and/or

other materials provided with the distribution.



Neither the name of Robert Atkinson nor the names of his contributors may be used to

endorse or promote products derived from this software without specific prior

written permission.



NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS

LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS

"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,

THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE

ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE

FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL

DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR

SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER

CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR

TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF

THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleOp--Omni Devin Use This", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

//@Disabled1

public class Teleop_Omni extends OpMode
{

    /* Declare OpMode members. */

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotorFront = null;
    private DcMotor rightMotorFront = null;
    private DcMotor rightMotorRear = null;
    private DcMotor leftMotorRear = null;
    private DcMotor liftMotor = null;
    private DcMotor liftLock = null;
    private DcMotor ballCollector = null;
    private DcMotor ballLift = null;
    Servo beefyArm;

    ColorSensor colorSensor;
    boolean cutSpeed;
    boolean cutLiftSpeed = false;
    boolean lockLiftLock = false;

    double continuousStop = .5 ;
    double horizontal; // variable for horizontal movement
    double vertical; // variable for vertical movement
    double rotational; // variable for rotational movement

    //sometimes, the trigger doesn't fully register that it is not being pressed, and it gives back a very small
    //number that will drain the motor. The deadzone prevents this. Any value than the deadzone value will be ignored.
    double deadZone = .1;
    float hsvValues[] = {0F, 0F, 0F};


    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    /*

     * Code to run ONCE when the driver hits INIT

     */

    @Override

    public void init() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Jenna:", "Don't forget to press play. It won't work if you don't press play");


        leftMotorFront  = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        rightMotorRear = hardwareMap.dcMotor.get("rightMotorRear");
        leftMotorRear = hardwareMap.dcMotor.get("leftMotorRear");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftLock = hardwareMap.dcMotor.get("liftLock");


        ballCollector = hardwareMap.dcMotor.get("ballCollector");
        ballLift = hardwareMap.dcMotor.get("ballLift");
        beefyArm = hardwareMap.servo.get("beefyArm");
        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery

        leftMotorFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotorRear.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotorFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotorRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        liftMotor.setDirection(DcMotor.Direction.FORWARD);


        //colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");



        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).

         */

    }



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override

    public void init_loop() {

    }



    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        /*
         * The idea is that the different variables represent a direction for the omni wheels.
         * For example, when the robot moves forward, the motors all move forward (the right
         * side is reversed). Therefore, the vertical variable is positive for all of the motors.
         * To go sideways, the front right and back left must move forward, while the other
         * two motors must move backwards, which is why the horizontal variable is negative for
         * two of the motors.
         */

        vertical = gamepad1.right_stick_y + gamepad1.left_stick_y; // vertical movement is controlled by up and down on the right stick
        horizontal = gamepad1.right_stick_x; // horizontal movement is controlled by side   to side on the right stick
        rotational = -gamepad1.left_stick_x; // pirouetting is controlled by side to side on the left stick

        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);


        //the triggers allow the driver to cut the speed of the robot in order to provide more control
        if (gamepad1.left_bumper && !cutSpeed){
            cutSpeed = true;

        } else if (gamepad1.right_bumper && cutSpeed){
            cutSpeed = false;

        }

        if (cutSpeed) {
            leftMotorFront.setPower((vertical - horizontal + rotational) / 3);
            leftMotorRear.setPower((vertical + horizontal + rotational) / 3);
            rightMotorFront.setPower((vertical + horizontal - rotational) / 3);
            rightMotorRear.setPower((vertical - horizontal - rotational) / 3);

        } else if (!cutSpeed) {
            leftMotorFront.setPower(vertical - horizontal + rotational);
            leftMotorRear.setPower(vertical + horizontal + rotational);
            rightMotorFront.setPower(vertical + horizontal - rotational);
            rightMotorRear.setPower(vertical - horizontal - rotational);

        }
/*
        if (gamepad2.left_bumper && !cutLiftSpeed){
            cutLiftSpeed = true;

        } else if (gamepad2.right_bumper && cutLiftSpeed){
            cutLiftSpeed = false;

        }

        if (cutLiftSpeed){
            liftMotor.setPower(gamepad2.right_stick_y/2);
        } else if (!cutLiftSpeed){
            liftMotor.setPower(gamepad2.right_stick_y);
        }

*/
        liftMotor.setPower(gamepad2.right_stick_y);
        ballLift.setPower(gamepad2.left_stick_y);


        if(gamepad1.b){
            lockLiftLock = true;
        } else if (gamepad1.a){
            lockLiftLock = false;
        }

        if(lockLiftLock){
            liftLock.setPower(1);
        }else {
            liftLock.setPower(0);
        }



        if (gamepad2.left_trigger > deadZone){
            ballCollector.setPower(-gamepad2.left_trigger);

        } else if (gamepad2.right_trigger > deadZone){
            ballCollector.setPower(gamepad2.right_trigger);

        } else {
            ballCollector.setPower(0);
        }
        /*

         * Code to run ONCE after the driver hits STOP

         */

    }

    @Override
    public void stop () {

    }

}