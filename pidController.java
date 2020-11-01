package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "PidTest", group = "6209")
public class pidController extends LinearOpMode {

    // Adding the necessary objects
    //IMU gyro = new IMU();
    Orientation gyro = new Orientation();
    HardwareTest robot = new HardwareTest();


    public void runOpMode(){


        // Add data to phone and wait for the start button to be pressed
        telemetry.addData(">", "Wait for Start");
        telemetry.update();
        waitForStart();


    }

    public void pidTurnLoop(double target, double turnPower){


        double current = 0;
        double error = target-current;
        double prevError = 0;

        double proportional;
        double integral = 0;
        double derivative = 0;

        double prevTime = 0;


        double kP = 0.6/90;
        double kI = 0.012;
        double kD = 0.02/90;


        while(Math.abs(error) >= 1){
            current = gyro.firstAngle;
            error = Math.abs(target - current);
            proportional = error*kP;
            integral += error * ((System.currentTimeMillis()/1000)-prevTime) * kI;
            derivative = (error - prevError) / ((System.currentTimeMillis()/1000) - prevTime) * kD;
            turnPower = proportional + integral + derivative;
        }

        if(gamepad1.right_stick_y > 0){
            robot.leftBackMotor.setPower(turnPower);
            robot.leftFrontMotor.setPower(turnPower);
            robot.rightFrontMotor.setPower(-turnPower);
            robot.rightBackMotor.setPower(-turnPower);
        }
        if(gamepad1.right_stick_y < 0){
            robot.leftBackMotor.setPower(-turnPower);
            robot.leftFrontMotor.setPower(-turnPower);
            robot.rightFrontMotor.setPower(turnPower);
            robot.rightBackMotor.setPower(turnPower);
        }
    }
}
