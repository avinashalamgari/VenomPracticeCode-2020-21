package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

@Autonomous(name = "autothing", group = "Auto")
public class AutoTest extends LinearOpMode {
    DcMotor leftBackMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor rightFrontMotor;
    DcMotor shooter;
    DcMotor conveyerBeltLeft;
    DcMotor conveyerBeltRight;

    @Override
    public void runOpMode() throws InterruptedException{
        leftBackMotor  = hardwareMap.get(DcMotor.class, "leftBackMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        conveyerBeltLeft = hardwareMap.get(DcMotor.class, "transitionBeltLeft");
        conveyerBeltRight = hardwareMap.get(DcMotor.class, "transitionBeltRight");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

//        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        // leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        // *** TRY WITHOUT REVERSING *** //
//        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
//        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
//
//
//
//        leftBackMotor.setPower(-1);
//        leftFrontMotor.setPower(1);
//        rightBackMotor.setPower(-1);
//        rightFrontMotor.setPower(1);
//
//        sleep(1000);
//
//        leftBackMotor.setPower(0);
//        leftFrontMotor.setPower(0);
//        rightBackMotor.setPower(0);
//        rightFrontMotor.setPower(0);
//
//        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        leftBackMotor.setTargetPosition(distance(4));
//        leftFrontMotor.setTargetPosition(distance(4));
//        rightFrontMotor.setTargetPosition(distance(4));
//        rightBackMotor.setTargetPosition(distance(4));
//
//        leftBackMotor.setPower(0.5);
//        leftFrontMotor.setPower(-0.5);
//        rightBackMotor.setPower(-0.5);
//        rightFrontMotor.setPower(0.5);
//
//
//
//        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        Thread.sleep(5000);
//
//        while(leftBackMotor.isBusy() || rightBackMotor.isBusy() || leftFrontMotor.isBusy() || rightFrontMotor.isBusy()){
//        leftBackMotor.setPower(0);
//        leftFrontMotor.setPower(0);
//        rightBackMotor.setPower(0);
//        rightFrontMotor.setPower(0);
        //strafe(true);

        //Thread.sleep(1000);

        forward();

        Thread.sleep(1350);

        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);

        Thread.sleep(1000);

        backwards();

        Thread.sleep(600);

        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);

        Thread.sleep(500);

        turn(true);
        Thread.sleep(25);

//        strafe(true);
//
//        Thread.sleep(500);

        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);

        Thread.sleep(1000);

        shooter.setPower(1);
        Thread.sleep(5000);

        shoot();
        Thread.sleep(3000);


        shooter.setPower(0);
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);

        forward();
        Thread.sleep(200);

        shooter.setPower(0);
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);

        }




    public int distance(double inches) {
        double diameter = 3.93701;                                // In Inches
        double circumference = (Math.PI)*(diameter);
        double rotationsNeeded = inches/circumference;
        int target = (int)rotationsNeeded * 1440;
        /*
         * Circumference = Distance traveled in 1 rotation
         * 1440 encoder ticks is 1 rotation
         */
        return target;
    }

    public void strafe(boolean right){
        if(right){
            leftFrontMotor.setPower(-0.5);
            leftBackMotor.setPower(-0.5);
            rightFrontMotor.setPower(0.5);
            rightBackMotor.setPower(0.5);
        }
    }

    public void forward(){
        leftFrontMotor.setPower(-0.8);
        leftBackMotor.setPower(0.8);
        rightFrontMotor.setPower(-0.8);
        rightBackMotor.setPower(0.8);
    }

    public void backwards(){
        leftFrontMotor.setPower(0.5);
        leftBackMotor.setPower(-0.5);
        rightFrontMotor.setPower(0.5);
        rightBackMotor.setPower(-0.5);
    }

    public void turn(boolean right){
        if(right){
            leftFrontMotor.setPower(-0.5);
            leftBackMotor.setPower(0.5);
            rightFrontMotor.setPower(0.5);
            rightBackMotor.setPower(-0.5);
        }
    }

    public void shoot() throws InterruptedException {
        conveyerBeltRight.setPower(-1);
        conveyerBeltLeft.setPower(1);

        Thread.sleep(50);

        conveyerBeltRight.setPower(0);
        conveyerBeltLeft.setPower(0);
        Thread.sleep(800);

        conveyerBeltRight.setPower(-1);
        conveyerBeltLeft.setPower(1);

        Thread.sleep(100);

        conveyerBeltRight.setPower(0);
        conveyerBeltLeft.setPower(0);
        Thread.sleep(500);

        conveyerBeltRight.setPower(-1);
        conveyerBeltLeft.setPower(1);

        Thread.sleep(100);

        conveyerBeltRight.setPower(0);
        conveyerBeltLeft.setPower(0);

    }
}
