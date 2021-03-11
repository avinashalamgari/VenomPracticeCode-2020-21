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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
@Autonomous(name = "Faster Final Auto Class", group = "Concept")
public class FasterFinalAutoClass extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

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



    DcMotor leftBackMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor rightFrontMotor;
    DcMotor shooter;
    DcMotor conveyerBeltLeft;
    DcMotor conveyerBeltRight;

    Servo horizontal;
    CRServo vertical;


    Orientation angles;
    private double ticksPerRev = 383.6;

    HardwareTest robot = new HardwareTest();

    public String label = "";

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
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


            leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
            leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

            leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
//                            if(recognition.getLabel().equals("")){
//                                label = "";
//                            }
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

        while (opModeIsActive()){
            telemetry.addData("Label: ", label);
            telemetry.update();

            gotoSquareA(0.7);
            gotoSquareB(0.7);
            gotoSquareC(0.7);
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
       tfodParameters.minResultConfidence = 0.5f;
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
    public void forward(double power){
        leftBackMotor.setPower(power);
        leftFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
        rightFrontMotor.setPower(power);
    }

    public void backward(double power){
        leftBackMotor.setPower(-power);
        leftFrontMotor.setPower(-power);
        rightBackMotor.setPower(-power);
        rightFrontMotor.setPower(-power);
    }

    public void turn(double power){
        leftBackMotor.setPower(power);
        leftFrontMotor.setPower(power);
        rightBackMotor.setPower(-power);
        rightFrontMotor.setPower(-power);
    }

    public void stopDriving(){
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

    public void strafe(double power){
        leftBackMotor.setPower(-power);
        leftFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
        rightFrontMotor.setPower(-power);
    }

    public void gotoSquareA(double power) throws InterruptedException {
        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        if(!label.equals("Single") && !label.equals("Quad")) {
            rightFrontMotor.setTargetPosition(distance(60));
            rightBackMotor.setTargetPosition(distance(60));
            leftBackMotor.setTargetPosition(distance(60));
            leftFrontMotor.setTargetPosition(distance(60));

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

            rightFrontMotor.setTargetPosition(-distance(24));
            rightBackMotor.setTargetPosition(distance(24));
            leftBackMotor.setTargetPosition(-distance(24));
            leftFrontMotor.setTargetPosition(distance(24));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            strafe(0.4);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            stopDriving();


            shoot();

            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            rightFrontMotor.setTargetPosition(distance(12));
            rightBackMotor.setTargetPosition(distance(12));
            leftBackMotor.setTargetPosition(distance(12));
            leftFrontMotor.setTargetPosition(distance(12));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            forward(power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);


            rightFrontMotor.setTargetPosition(-distance(43));
            rightBackMotor.setTargetPosition(-distance(43));
            leftBackMotor.setTargetPosition(distance(43));
            leftFrontMotor.setTargetPosition(distance(43));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            turn(0.7);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            stopDriving();

            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);


            rightFrontMotor.setTargetPosition(-distance(30));
            rightBackMotor.setTargetPosition(distance(30));
            leftBackMotor.setTargetPosition(-distance(30));
            leftFrontMotor.setTargetPosition(distance(30));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            strafe(power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }


            stopDriving();

            vertical.setPower(-0.1);
            Thread.sleep(1200);
            vertical.setPower(0);


            horizontal.setPosition(0.5);
        }
    }

    public void gotoSquareB(double power) throws InterruptedException {
        if(label.equals("Single")) {
            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            rightFrontMotor.setTargetPosition(distance(58));
            rightBackMotor.setTargetPosition(distance(58));
            leftBackMotor.setTargetPosition(distance(58));
            leftFrontMotor.setTargetPosition(distance(58));

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

            rightFrontMotor.setTargetPosition(-distance(24));
            rightBackMotor.setTargetPosition(distance(24));
            leftBackMotor.setTargetPosition(-distance(24));
            leftFrontMotor.setTargetPosition(distance(24));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            strafe(0.4);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            stopDriving();


            shoot();


            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            rightFrontMotor.setTargetPosition(distance(29));
            rightBackMotor.setTargetPosition(distance(29));
            leftBackMotor.setTargetPosition(distance(29));
            leftFrontMotor.setTargetPosition(distance(29));

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


            rightFrontMotor.setTargetPosition(distance(32));
            rightBackMotor.setTargetPosition(distance(32));
            leftBackMotor.setTargetPosition(-distance(32));
            leftFrontMotor.setTargetPosition(-distance(32));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            turn(-0.7);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            rightFrontMotor.setTargetPosition(-distance(5));
            rightBackMotor.setTargetPosition(distance(5));
            leftBackMotor.setTargetPosition(-distance(5));
            leftFrontMotor.setTargetPosition(distance(5));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            strafe(0.4);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            vertical.setPower(-0.1);
            Thread.sleep(1200);
            vertical.setPower(0);
            horizontal.setPosition(0.5);


            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            rightFrontMotor.setTargetPosition(distance(24));
            rightBackMotor.setTargetPosition(distance(24));
            leftBackMotor.setTargetPosition(distance(24));
            leftFrontMotor.setTargetPosition(distance(24));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            forward(power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }
        }
    }

    public void gotoSquareC(double power) throws InterruptedException {
        if(label.equals("Quad")){
            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            rightFrontMotor.setTargetPosition(distance(60));
            rightBackMotor.setTargetPosition(distance(60));
            leftBackMotor.setTargetPosition(distance(60));
            leftFrontMotor.setTargetPosition(distance(60));

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

            rightFrontMotor.setTargetPosition(-distance(24));
            rightBackMotor.setTargetPosition(distance(24));
            leftBackMotor.setTargetPosition(-distance(24));
            leftFrontMotor.setTargetPosition(distance(24));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            strafe(0.4);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            stopDriving();


            shoot();

            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            rightFrontMotor.setTargetPosition(distance(48));
            rightBackMotor.setTargetPosition(distance(48));
            leftBackMotor.setTargetPosition(distance(48));
            leftFrontMotor.setTargetPosition(distance(48));

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

            rightFrontMotor.setTargetPosition(distance(36));
            rightBackMotor.setTargetPosition(-distance(36));
            leftBackMotor.setTargetPosition(distance(36));
            leftFrontMotor.setTargetPosition(-distance(36));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            strafe(-0.7);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);


            rightFrontMotor.setTargetPosition(distance(21.5));
            rightBackMotor.setTargetPosition(distance(21.5));
            leftBackMotor.setTargetPosition(-distance(21.5));
            leftFrontMotor.setTargetPosition(-distance(21.5));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            turn(-.3);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }

            stopDriving();

            vertical.setPower(-0.1);
            Thread.sleep(1200);
            vertical.setPower(0);

            horizontal.setPosition(0.5);

            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);


            rightFrontMotor.setTargetPosition(distance(21.5));
            rightBackMotor.setTargetPosition(distance(21.5));
            leftBackMotor.setTargetPosition(-distance(21.5));
            leftFrontMotor.setTargetPosition(-distance(21.5));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            turn(-.3);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }


            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);


            rightFrontMotor.setTargetPosition(distance(60-12));
            rightBackMotor.setTargetPosition(distance(60-12));
            leftBackMotor.setTargetPosition(distance(60-12));
            leftFrontMotor.setTargetPosition(distance(60-12));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            forward(power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {
            }

            stopDriving();

        }
    }

    private void shoot() throws InterruptedException {

        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);


        rightFrontMotor.setTargetPosition(-distance(3.5));
        rightBackMotor.setTargetPosition(-distance(3.5));
        leftBackMotor.setTargetPosition(distance(3.5));
        leftFrontMotor.setTargetPosition(distance(3.5));

        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turn(0.3);

        while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

        }

        stopDriving();

        shooter.setPower(1);
        Thread.sleep(5000);

        conveyerBeltRight.setPower(-0.5);
        conveyerBeltLeft.setPower(0.5);

        Thread.sleep(50);


        conveyerBeltRight.setPower(0);
        conveyerBeltLeft.setPower(0);
        Thread.sleep(800);

        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);


        rightFrontMotor.setTargetPosition(-distance(1));
        rightBackMotor.setTargetPosition(-distance(1));
        leftBackMotor.setTargetPosition(distance(1));
        leftFrontMotor.setTargetPosition(distance(1));

        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turn(0.3);

        while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

        }

        stopDriving();

        conveyerBeltRight.setPower(-1);
        conveyerBeltLeft.setPower(1);

        Thread.sleep(50);



        conveyerBeltRight.setPower(0);
        conveyerBeltLeft.setPower(0);
        Thread.sleep(100);

        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);


        rightFrontMotor.setTargetPosition(-distance(1));
        rightBackMotor.setTargetPosition(-distance(1));
        leftBackMotor.setTargetPosition(distance(1));
        leftFrontMotor.setTargetPosition(distance(1));

        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turn(0.3);

        while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

        }

        stopDriving();

        conveyerBeltRight.setPower(-1);
        conveyerBeltLeft.setPower(1);

        Thread.sleep(50);

        conveyerBeltRight.setPower(0);
        conveyerBeltLeft.setPower(0);

        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);


        rightFrontMotor.setTargetPosition(distance(7));
        rightBackMotor.setTargetPosition(distance(7));
        leftBackMotor.setTargetPosition(-distance(7));
        leftFrontMotor.setTargetPosition(-distance(7));

        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turn(-0.3);

        while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

        }

        conveyerBeltRight.setPower(0);
        conveyerBeltLeft.setPower(0);

        shooter.setPower(0);

        stopDriving();
    }
}
