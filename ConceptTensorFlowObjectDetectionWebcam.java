/* Copyright (c) 2019 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Concept: TensorFlow Object Detection Webcam", group = "Concept")

public class ConceptTensorFlowObjectDetectionWebcam extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    HardwareTest robot = new HardwareTest();
    double horizontalPosition = robot.HORIZONTAL_HOME;
    final double HORIZONTAL_ARM_SPEED = 0.01;

    DcMotor leftBackMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor rightFrontMotor;
    DcMotor shooter;
    DcMotor conveyerBeltLeft;
    DcMotor conveyerBeltRight;

    Servo horizontal;
    CRServo vertical;

    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    Orientation angles;
    private double ticksPerRev = 383.6;



    public String label = "";

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
    private static final String VUFORIA_KEY =
            "Aa2xRpz/////AAABmV79umPuRUzvmnnQYhoUglJjcW8UHfatFVoYYt4vWQIJUTiz0VtR0wcSxJYAxe3uVAS9F3Jsn82zu+8mFQJ3pyxD8+fMuRMokYTCP9XOQ63eFi08JKhYo3YnSCowNrxOL6VMRjaI+32NcBgSqf39+fmjQ67N7tUtipe73NXfF4B5MmJE+4VCoUrhKGqnOhqCZk2Q5RTfhNu089+lxRlDWlhXtM1+yyVhPg5mbg3OEHE6K9q+GSAxPj+41D9FWpt8rKAixdAaal29lpYe5n4eC+DXfpDUsUXEkAb40P242XbFxXGrUK11J23ZDq83SgB1czIuWITRTpZijwl1+QNKcdLxWOHYqB3z585bmnInHBIi";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

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
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

            if (!opModeIsActive()) {
            while (!opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    label = updatedRecognitions.get(updatedRecognitions.size()).getLabel();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            label = recognition.getLabel();
                            telemetry.addData("Label variable", label);
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                }
            }
        }




        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//                if (tfod != null) {
//                    // getUpdatedRecognitions() will return null if no new information is available since
//                    // the last time that call was made.
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
////                    label = updatedRecognitions.get(updatedRecognitions.size()).getLabel();
//                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Object Detected", updatedRecognitions.size());
//                        // step through the list of recognitions and display boundary info.
//                        int i = 0;
//                        for (Recognition recognition : updatedRecognitions) {
//                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                            label = recognition.getLabel();
//                            telemetry.addData("Label variable", label);
//                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                    recognition.getLeft(), recognition.getTop());
//                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                                    recognition.getRight(), recognition.getBottom());
//                        }
//                        telemetry.update();
//                    }
//                }
//            }
//        }

        while(opModeIsActive()){
            telemetry.addData("Label variable", label);
            runCode();

            horizontal.setPosition(0);
            stop();
        }

        if (tfod != null) {
            tfod.shutdown();
        }


    }

    /**
     * Initialize the Vuforia localization engine.
     */
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
       tfodParameters.minResultConfidence = 0.45f;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    public int distance(double inches) {
        double diameter = 3.93701;                                // In Inches
        double circumference = (Math.PI)*(diameter);
        double rotationsNeeded = inches/circumference;
        double target = rotationsNeeded * ticksPerRev;
        /*
         * Circumference = Distance traveled in 1 rotation
         * 1440 encoder ticks is 1 rotation
         */
        return (int)target;
    }

    //    public void strafe(boolean right){
//        if(right){
//            leftFrontMotor.setPower(-0.5);
//            leftBackMotor.setPower(-0.5);
//            rightFrontMotor.setPower(0.5);
//            rightBackMotor.setPower(0.5);
//        }
//    }
//
//    public void forward(){
//        leftFrontMotor.setPower(-0.8);
//        leftBackMotor.setPower(0.8);
//        rightFrontMotor.setPower(-0.8);
//        rightBackMotor.setPower(0.8);
//    }
//
//    public void backwards(){
//        leftFrontMotor.setPower(0.5);
//        leftBackMotor.setPower(-0.5);
//        rightFrontMotor.setPower(0.5);
//        rightBackMotor.setPower(-0.5);
//    }
//
    public void turn(boolean right, double power){
        if(right){
            leftFrontMotor.setPower(-power);
            leftBackMotor.setPower(power);
            rightFrontMotor.setPower(power);
            rightBackMotor.setPower(-power);
        }
    }
    //
    public void shoot() throws InterruptedException {
        conveyerBeltRight.setPower(-1);
        conveyerBeltLeft.setPower(1);

        Thread.sleep(50);

        conveyerBeltRight.setPower(0);
        conveyerBeltLeft.setPower(0);
        Thread.sleep(800);

        conveyerBeltRight.setPower(-1);
        conveyerBeltLeft.setPower(1);

        Thread.sleep(50);



        conveyerBeltRight.setPower(0);
        conveyerBeltLeft.setPower(0);
        Thread.sleep(500);


        conveyerBeltRight.setPower(-1);
        conveyerBeltLeft.setPower(1);

        Thread.sleep(50);

        conveyerBeltRight.setPower(0);
        conveyerBeltLeft.setPower(0);


    }
    public void forward(double power){
        leftBackMotor.setPower(power);
        leftFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
        rightFrontMotor.setPower(power);
    }

    public void backward(double power){
        leftBackMotor.setPower(-power);
        leftFrontMotor.setPower(power);
        rightBackMotor.setPower(-power);
        rightFrontMotor.setPower(power);
    }

    public void driveForwardDistance(double power, int distance){


        rightFrontMotor.setTargetPosition(-distance);
        rightBackMotor.setTargetPosition(distance);
        leftFrontMotor.setTargetPosition(-distance);
        leftBackMotor.setTargetPosition(distance);

        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        forward(power);

        while(leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()){

        }

        stopDriving();
    }

//    public void turn (double turnPower){
//        leftBackMotor.setPower(turnPower);
//        leftFrontMotor.setPower(-turnPower);
//        rightBackMotor.setPower(-turnPower);
//        rightFrontMotor.setPower(turnPower);
//    }

    public void goToSquareA(double power) throws InterruptedException {

        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        if(!label.equals("Single") && !label.equals("Quad")) {
            rightFrontMotor.setTargetPosition(distance(72));
            rightBackMotor.setTargetPosition(distance(72));
            leftBackMotor.setTargetPosition(distance(72));
            leftFrontMotor.setTargetPosition(distance(72));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            forward(power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            stopDriving();

//            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
//            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
//            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
//            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
//
//            rightFrontMotor.setTargetPosition(distance(48));
//            rightBackMotor.setTargetPosition(-distance(48));
//            leftBackMotor.setTargetPosition(distance(48));
//            leftFrontMotor.setTargetPosition(-distance(48));
//
//
//            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            turn(true, power);
//
//            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {
//
//            }
            rotate(180, 0.1);

            vertical.setPower(-0.1);
            Thread.sleep(800);
            vertical.setPower(0);

            horizontal.setPosition(0.5);

            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            rightFrontMotor.setTargetPosition(distance(48));
            rightBackMotor.setTargetPosition(-distance(48));
            leftBackMotor.setTargetPosition(distance(48));
            leftFrontMotor.setTargetPosition(-distance(48));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            turn(true, power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            rightFrontMotor.setTargetPosition(distance(24));
            rightBackMotor.setTargetPosition(-distance(24));
            leftBackMotor.setTargetPosition(-distance(24));
            leftFrontMotor.setTargetPosition(distance(24));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            backward(power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            stopDriving();

            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            Thread.sleep(500);

            rightFrontMotor.setTargetPosition(distance(24));
            rightBackMotor.setTargetPosition(-distance(24));
            leftBackMotor.setTargetPosition(distance(24));
            leftFrontMotor.setTargetPosition(-distance(24));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            turn(true, power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            stopDriving();
        }
    }

    public void goToSquareB(double power) throws InterruptedException {

        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        if(label.equals("Single")) {
            rightFrontMotor.setTargetPosition(-distance(96));
            rightBackMotor.setTargetPosition(distance(96));
            leftBackMotor.setTargetPosition(distance(96));
            leftFrontMotor.setTargetPosition(-distance(96));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            forward(power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            stopDriving();

            vertical.setPower(-0.1);
            Thread.sleep(800);
            vertical.setPower(0);

            horizontal.setPosition(0.5);

            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            rightFrontMotor.setTargetPosition(distance(24));
            rightBackMotor.setTargetPosition(-distance(24));
            leftBackMotor.setTargetPosition(-distance(24));
            leftFrontMotor.setTargetPosition(distance(24));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            forward(power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            stopDriving();


            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            Thread.sleep(500);

            rightFrontMotor.setTargetPosition(distance(24));
            rightBackMotor.setTargetPosition(-distance(24));
            leftBackMotor.setTargetPosition(distance(24));
            leftFrontMotor.setTargetPosition(-distance(24));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            turn(true, power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            stopDriving();
        }
    }

    public void goToSquareC(double power) throws InterruptedException {
        if(label.equals("Quad")) {
            rightFrontMotor.setTargetPosition(-distance(120));
            rightBackMotor.setTargetPosition(distance(120));
            leftBackMotor.setTargetPosition(distance(120));
            leftFrontMotor.setTargetPosition(-distance(120));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            forward(power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            stopDriving();

            vertical.setPower(-0.1);
            Thread.sleep(800);
            vertical.setPower(0);

            horizontal.setPosition(0.5);



            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            Thread.sleep(500);

            rightFrontMotor.setTargetPosition(distance(24));
            rightBackMotor.setTargetPosition(-distance(24));
            leftBackMotor.setTargetPosition(distance(24));
            leftFrontMotor.setTargetPosition(-distance(24));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            turn(true, power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            stopDriving();
        }
    }

    public void runCode() throws InterruptedException {


        leftBackMotor  = hardwareMap.get(DcMotor.class, "leftBackMotor");
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyerBeltLeft = hardwareMap.get(DcMotor.class, "transitionBeltLeft");
        conveyerBeltRight = hardwareMap.get(DcMotor.class, "transitionBeltRight");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        vertical = hardwareMap.crservo.get("servo");

        horizontal = hardwareMap.servo.get("servo2");
        horizontal.setPosition(robot.HORIZONTAL_HOME);



        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("distance Method", distance(12));
        telemetry.update();
        //driveForwardDistance(0.5, distance(60));

        //goToSquareC(1);
        telemetry.addData("Label",label);
        telemetry.update();
        goToSquareA(0.5);
        goToSquareB(0.5);
        goToSquareC(0.5);

        stopDriving();
    }

    public void stopDriving(){
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        leftFrontMotor.setPower(-leftPower);
        leftBackMotor.setPower(leftPower);
        rightFrontMotor.setPower(rightPower);
        rightBackMotor.setPower(-rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        stopDriving();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}
