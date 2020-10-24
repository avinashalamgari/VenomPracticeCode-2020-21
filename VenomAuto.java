package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import java.lang.System;
import java.util.Set;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;

public class VenomAuto {

    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;

    SensorBNO055IMU gyro = new SensorBNO055IMU();
    HardwareTest robot = new HardwareTest();
    //private Servo arm;


    public void init(){
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");


        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void turn(double turnPower){
        if(gamepad1.right_stick_y > 0){
            leftBackMotor.setPower(turnPower);
            leftFrontMotor.setPower(turnPower);
            rightFrontMotor.setPower(-turnPower);
            rightBackMotor.setPower(-turnPower);
        }
        if(gamepad1.right_stick_y < 0){
            leftBackMotor.setPower(-turnPower);
            leftFrontMotor.setPower(-turnPower);
            rightFrontMotor.setPower(turnPower);
            rightBackMotor.setPower(turnPower);
        }
    }

    public void pidTurn(double target){

        double turnPower;

        double current = 0;
        double error = target-current;
        double prevError = 0;

        double proportional;
        double integral = 0;
        double derivative

        double prevTime = 0;


        double kP = 0.6/90;
        double kI = 0.012;
        double kD = 0.02/90;


        while(Math.abs(error) >= 1){
            current = gyro;            //fix error (no gryo method exists)
            error = Math.abs(target - current);
            proportional = error*kP;
            integral += error * ((System.currentTimeMillis()/1000)-prevTime) * kI;
            derivative = (error - prevError) / ((System.currentTimeMillis()/1000) - prevTime) * kD;
            turnPower = proportional + integral + derivative;
            turn(turnPower);
        }
    }


}
