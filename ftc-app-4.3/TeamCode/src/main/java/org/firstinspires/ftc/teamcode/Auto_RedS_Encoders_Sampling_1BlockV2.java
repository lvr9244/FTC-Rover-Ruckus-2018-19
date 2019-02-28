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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


@Autonomous(name="Auto Square Encoders Sampling 1 block V2", group ="Concept")
//@Disabled
    public class Auto_RedS_Encoders_Sampling_1BlockV2 extends LinearOpMode {

//hello paul
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 1680;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // or figuring circumference
    double pi = 3.14159265358979;
    double turnFourtyFive = (15.78*pi)/8;
    double turnNinety = (17.78*pi)/4;
    double turn180 = (17.78*pi)/2;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private TFObjectDetector tfod;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    boolean pictureVisible = false;
    boolean mineralFound = false;
    boolean goldLeft = false;
    boolean goldRight = false;
    boolean goldMiddle = false;
    boolean lockLiftMotor = false;
    boolean scanOne = false;
    boolean scanTwo = false;

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


    private DcMotor leftMotorFront = null;
    private DcMotor rightMotorFront = null;
    private DcMotor rightMotorRear = null;
    private DcMotor leftMotorRear = null;
    private DcMotor liftMotor = null;

    Servo beefyArm;

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
        liftMotor = hardwareMap.dcMotor.get("liftMotor");

        beefyArm = hardwareMap.servo.get("beefyArm");




        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery

        leftMotorFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotorRear.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotorFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotorRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);


        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);


        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);


        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT = 125;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 230;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 15;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
            telemetry.addLine("tFod activated");
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.update();



        //Start actual game code
        waitForStart();


        //this next sequence drops the robot down
        liftMotor.setPower(-1);
        sleep(500);
        liftMotor.setPower(0);

        telemetry.addLine("motor moved down");
        telemetry.update();
        sleep(2000);

        //unhook ourselves
        encoderDrive(1, -5, 5, 5, -5, 30);
        liftMotor.setPower(1);
        sleep(300);
        liftMotor.setPower(0);
        encoderDrive(1, -3, 3, 3, -3, 30);
        encoderDrive(1, 8, 8, 8, 8 , 30);
        encoderDrive(1, 3, -3, -3, 3, 30);

        //encoderDrive(1, -8, 8, 8, -8, 30);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
                telemetry.addLine("tensor flow activated");
                sleep(500);
            }

            //this is our first scan. until we declare scanOne = true (a ball was seen) the loop will continue.

            while (!scanOne){
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                         if (updatedRecognitions.size() == 1) {
                            telemetry.addLine("Object Detected");
                            motorOff();
                            telemetry.update();
                            sleep(3000);
                            for (Recognition recognition : updatedRecognitions) {
                                //if gold is found, then the mineralFound = true. this will prevent the robot from doing a second
                                //scan, as it isn't necessary.
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    telemetry.addData("Gold Found", ".");
                                    goldMiddle = true;
                                    mineralFound = true;
                                    scanOne = true;
                                    telemetry.addLine("mineral found true");

                                } else {
                                    telemetry.addLine("Silver Found");
                                    scanOne = true;
                                }
                            }
                        } //here, if a ball is not seen, we move very slowly to the left until we recognize a ball.
                        else {
                            driveRight(.1);
                        }
                    }
                }
            } telemetry.addLine("First scan done");
            telemetry.update();
            sleep(1000);

        }

        //this is scan two. if we do not find a gold block from the first scan, we will do a second scan on the far right ball
        if (!mineralFound){
            encoderDrive(1, 14, -14, -14, 14  , 30);
            while (!scanTwo){
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 1) {
                            telemetry.addLine("Object Detected");
                            motorOff();
                            telemetry.update();
                            sleep(1000);
                            for (Recognition recognition : updatedRecognitions) {
                                //if gold is found, then the mineralFound = true. this will prevent the robot from doing a second
                                //scan, as it isn't necessary.
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    telemetry.addData("Gold Found", ".");
                                    goldRight = true;
                                    mineralFound = true;
                                    scanTwo = true;
                                    telemetry.addLine("mineral found true");

                                } else {
                                    telemetry.addLine("Silver Found");
                                    scanTwo = true;
                                    goldLeft = true;
                                }
                            }
                        } //here, if a ball is not seen, we move very slowly to the left until we recognize a ball.
                        else {
                            driveRight(.1);
                        }
                    }
                }
            }
        }

        sleep(500);
        //using the booleans given by runTfod, sample the gold block and move to depot
        if (goldLeft){
            encoderDrive(1, -30, 30, 30, -30, 30);
            encoderDrive(1,  50, 50, 50,    50, 45);
            encoderDrive(1, -turnFourtyFive, turnFourtyFive, -turnFourtyFive, turnFourtyFive, 30);
            encoderDrive(1, 15, 15, 15, 15, 30);

        } else if (goldMiddle){
            encoderDrive(1, 60, 60, 60, 60, 45);
            encoderDrive(1, -turnFourtyFive, turnFourtyFive, -turnFourtyFive, turnFourtyFive, 30);

        } else if (goldRight){
            //encoderDrive(1, 10, -10, -10, 10, 30);
            encoderDrive(1, 45, 45, 45, 45, 45);
        } else {
            encoderDrive(1, 45, 45, 45, 45, 45);
            encoderDrive(1, turnFourtyFive, -turnFourtyFive, turnFourtyFive, -turnFourtyFive, 30);
            encoderDrive(1, 10, 10, 10, 10, 30);
        }

        //the robot spins around so the beefy arm is facing into the depot
        encoderDrive(1, turn180, -turn180, turn180, -turn180, 30);
        //encoderDrive(1,8, -8, -8, 8, 30);


        //beefy arm comes out, drops beefy shark, and closes
        beefyArm.setPosition(1);
        sleep(1000);

        encoderDrive(1, 3, 3, 3, 3, 30);
        beefyArm.setPosition(0);
        sleep(500);

        encoderDrive(1, -turnFourtyFive, turnFourtyFive, -turnFourtyFive, turnFourtyFive, 30);

        //encoderDrive(1, turn180, -turn180, turn180, -turn180, 30);


        //move to crater
        //encoderDrive(1, -55, 55, 55, -55, 75);
        //encoderDrive(1, 5, 5, 5, 5, 30);
        //encoderDrive(1, -35, 35, 35, -35, 30);

        //turn so we can climb over the crater
        //encoderDrive(1, -turnNinety, turnNinety, -turnNinety, turnNinety, 30);
        //encoderDrive(1, -20, -20, -20, -20, 30);

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

            sleep(250);   // optional pause after each move

        }
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    private void runTfod(){

        runtime.reset();

        //the robot will break out of this loop when it sees the minerals(making a boolean true) or the runtime passes the allotted time
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 1) {
                    telemetry.addLine("Object Detected");
                    telemetry.update();
                    sleep(3000);
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            telemetry.addData("Gold Found", ".");
                            goldLeft = true;
                            mineralFound = true;
                            telemetry.addLine("mineral found true");

                        } else{
                            encoderDrive(1, 5, -5, -5, 5, 30);
                            telemetry.addLine("Silver Found");
                        }
                    }

                }
                telemetry.addLine("First scan done");
                telemetry.update();
            } sleep(5000);

            if (updatedRecognitions != null && !mineralFound) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 1) {
                    //int goldMineralX = -1;
                    int silverMineral1X = -1;
                    //int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            telemetry.addData("Gold Found", ".");
                            goldMiddle = true;

                        } else{
                            telemetry.addLine("Silver Found");
                            goldRight = true;


                        }
                    }

                }
                telemetry.update();
            }

        }
        telemetry.update();
        sleep(500);


    }


}




