package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Auto With PowerShot Encoders", group = "Auto")
public class AutoEncodersWithPowerShots extends LinearOpMode {
    DcMotor leftBackMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor rightFrontMotor;
    DcMotor shooter;
    DcMotor conveyerBeltLeft;
    DcMotor conveyerBeltRight;

    BNO055IMU imu;

    Orientation angles;
    private double ticksPerRev = 383.6;

    @Override
    public void runOpMode() throws InterruptedException{
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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);


        telemetry.addData("wait for start", "Wait for Start");
        telemetry.update();
        waitForStart();
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("distance Method", distance(12));
        telemetry.update();
        driveForwardDistance(0.5, distance(60));


        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Thread.sleep(1000);
        turn(true);
        Thread.sleep(100);

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        Thread.sleep(1000);

        shooter.setPower(1);
        Thread.sleep(5000);

        shoot();
        Thread.sleep(3000);


        shooter.setPower(0);

        stopDriving();

        forward(0.5);
        Thread.sleep(500);

        stopDriving();

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
    public void turn(boolean right){
        if(right){
            leftFrontMotor.setPower(-0.5);
            leftBackMotor.setPower(0.5);
            rightFrontMotor.setPower(0.5);
            rightBackMotor.setPower(-0.5);
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

        turn(true);
        Thread.sleep(55);
        stopDriving();

        Thread.sleep(1000);

        conveyerBeltRight.setPower(-1);
        conveyerBeltLeft.setPower(1);

        Thread.sleep(50);



        conveyerBeltRight.setPower(0);
        conveyerBeltLeft.setPower(0);
        Thread.sleep(500);

        turn(true);
        Thread.sleep(70);
        stopDriving();

        Thread.sleep(1000);

        conveyerBeltRight.setPower(-1);
        conveyerBeltLeft.setPower(1);

        Thread.sleep(50);

        conveyerBeltRight.setPower(0);
        conveyerBeltLeft.setPower(0);


    }
    public void forward(double power){
        leftBackMotor.setPower(power);
        leftFrontMotor.setPower(-power);
        rightBackMotor.setPower(power);
        rightFrontMotor.setPower(-power);
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


    public void stopDriving(){
        forward(0);
    }
}
