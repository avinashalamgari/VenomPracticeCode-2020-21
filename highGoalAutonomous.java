/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "highGoalAutonomous", group = "auto")
public class highGoalAutonomous extends LinearOpMode
{
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;

    DcMotor leftBackMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor rightFrontMotor;
    DcMotor shooter;
    DcMotor conveyerBeltLeft;
    DcMotor conveyerBeltRight;

    Servo horizontal;
    CRServo vertical;


    private double ticksPerRev = 383.6;

    HardwareTest robot = new HardwareTest();



    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        if(!opModeIsActive()){
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
        }

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            gotoSquareA(0.7);
            gotoSquareB(0.7);
            gotoSquareC(0.7);
            stop();
        }
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

        if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE) {
            rightFrontMotor.setTargetPosition(distance(56));
            rightBackMotor.setTargetPosition(distance(56));
            leftBackMotor.setTargetPosition(distance(56));
            leftFrontMotor.setTargetPosition(distance(56));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            forward(power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }


//            rightFrontMotor.setTargetPosition(-distance(5));
//            rightBackMotor.setTargetPosition(-distance(5));
//            leftBackMotor.setTargetPosition(distance(5));
//            leftFrontMotor.setTargetPosition(distance(5));
//
//            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            turn(0.7);
//
//            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {
//
//            }
//            stopDriving();

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

            rightFrontMotor.setTargetPosition(distance(18));
            rightBackMotor.setTargetPosition(distance(18));
            leftBackMotor.setTargetPosition(distance(18));
            leftFrontMotor.setTargetPosition(distance(18));

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

            rightFrontMotor.setTargetPosition(-distance(42));
            rightBackMotor.setTargetPosition(distance(42));
            leftBackMotor.setTargetPosition(-distance(42));
            leftFrontMotor.setTargetPosition(distance(42));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            strafe(power);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }


            stopDriving();

            vertical.setPower(-0.1);
            Thread.sleep(2000);
            vertical.setPower(0);


            horizontal.setPosition(0.5);
            stop();
        }
    }

    public void gotoSquareB(double power) throws InterruptedException {
        if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
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


            rightFrontMotor.setTargetPosition(distance(29));
            rightBackMotor.setTargetPosition(distance(29));
            leftBackMotor.setTargetPosition(-distance(29));
            leftFrontMotor.setTargetPosition(-distance(29));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            turn(-0.7);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }
            stopDriving();

            vertical.setPower(-0.1);
            Thread.sleep(2000);
            vertical.setPower(0);
            horizontal.setPosition(0.5);


            rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            rightFrontMotor.setTargetPosition(-distance(-24));
            rightBackMotor.setTargetPosition(distance(-24));
            leftBackMotor.setTargetPosition(-distance(-24));
            leftFrontMotor.setTargetPosition(distance(-24));

            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            strafe(-0.4);

            while (leftBackMotor.isBusy() || rightBackMotor.isBusy() || rightFrontMotor.isBusy() || leftFrontMotor.isBusy()) {

            }
            stopDriving();
        }
    }

    public void gotoSquareC(double power) throws InterruptedException {
        if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR){
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

            rightFrontMotor.setTargetPosition(distance(52));
            rightBackMotor.setTargetPosition(distance(52));
            leftBackMotor.setTargetPosition(distance(52));
            leftFrontMotor.setTargetPosition(distance(52));

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
            Thread.sleep(2000);
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


            rightFrontMotor.setTargetPosition(distance(36));
            rightBackMotor.setTargetPosition(distance(36));
            leftBackMotor.setTargetPosition(distance(36));
            leftFrontMotor.setTargetPosition(distance(36));

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

        stopDriving();

        shooter.setPower(1);
        Thread.sleep(5000);

        conveyerBeltRight.setPower(-0.5);
        conveyerBeltLeft.setPower(0.5);

        Thread.sleep(50);


        conveyerBeltRight.setPower(0);
        conveyerBeltLeft.setPower(0);
        Thread.sleep(800);


        stopDriving();

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


        shooter.setPower(0);
        stopDriving();
    }
}


