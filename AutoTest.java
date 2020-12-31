package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

@Autonomous
public class AutoTest extends LinearOpMode {
    DcMotor leftBackMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor rightFrontMotor;

    @Override
    public void runOpMode() throws InterruptedException{
        leftBackMotor  = hardwareMap.get(DcMotor.class, "leftBackDrive");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackDrive");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontDrive");

        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

//        // *** TRY WITHOUT REVERSING *** //
////        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
////        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
////
////
////
////        leftBackMotor.setPower(-1);
////        leftFrontMotor.setPower(1);
////        rightBackMotor.setPower(-1);
////        rightFrontMotor.setPower(1);
////
////        sleep(1000);
////
////        leftBackMotor.setPower(0);
////        leftFrontMotor.setPower(0);
////        rightBackMotor.setPower(0);
////        rightFrontMotor.setPower(0);

        leftBackMotor.setTargetPosition(distance(0));
        leftFrontMotor.setTargetPosition(distance(0));
        rightFrontMotor.setTargetPosition(distance(0));
        rightBackMotor.setTargetPosition(distance(0));
    }

    public int distance(double inches) {
        double diameter = 5;                                // In Inches
        double circumference = (Math.PI)*(diameter);
        double rotationsNeeded = inches/circumference;
        int target = (int)rotationsNeeded * 1440;
        /*
        * Circumference = Distance traveled in 1 rotation
        * 1440 encoder ticks is 1 rotation
        *
        * */
        return target;
    }
}
