package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OrientationSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Set;


@Autonomous(name = "PidTest", group = "6209")
public class Venom2021Auto extends LinearOpMode {


    HardwareTest robot = new HardwareTest();
    IMU imu = new IMU();

    public void runOpMode(){


        // Add data to phone and wait for the start button to be pressed
        telemetry.addData(">", "Wait for Start");
        telemetry.update();
        waitForStart();
        pidTurnLoop(90);


    }

    public void pidTurnLoop(double target){

        double turnPower = 0;

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
            current = getCurrentAngle();              // Gets the angle that the robot is currently at
            error = Math.abs(target - current);
            proportional = error * kP;
            integral += error * ((System.currentTimeMillis()/1000)-prevTime) * kI;
            derivative = (error - prevError) / ((System.currentTimeMillis()/1000) - prevTime) * kD;
            turnPower = proportional + integral + derivative;
        }
            robot.leftBackMotor.setPower(turnPower);
            robot.leftFrontMotor.setPower(turnPower);
            robot.rightFrontMotor.setPower(-turnPower);
            robot.rightBackMotor.setPower(-turnPower);

            telemetry.addData("Turn Power", turnPower);
    }
    public double getCurrentAngle(){
        Orientation angles = imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu.imu.getPosition();
        return angles.firstAngle;
    }

    public void goForward(double timeInSeconds){
        double drivePower = 0;
        double currentTime = 0;
        while(currentTime < timeInSeconds){
            drivePower = 1.0;
            robot.leftBackMotor.setPower(drivePower);
            robot.leftFrontMotor.setPower(drivePower);
            robot.rightFrontMotor.setPower(drivePower);
            robot.rightBackMotor.setPower(drivePower);
            telemetry.addData("Current Time", currentTime);
            currentTime += System.currentTimeMillis()/1000;
        }
        drivePower = 0;
        robot.leftBackMotor.setPower(drivePower);
        robot.leftFrontMotor.setPower(drivePower);
        robot.rightFrontMotor.setPower(drivePower);
        robot.rightBackMotor.setPower(drivePower);


    }
}
