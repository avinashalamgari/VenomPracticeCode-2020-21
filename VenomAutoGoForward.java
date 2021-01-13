package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "AutoForwardTest", group = "6209")
public class VenomAutoGoForward {
    HardwareTest robot = new HardwareTest();
    public void init(){
        robot.rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    public void runOpMode(){
        goForward(1);
    }
    public void goForward(double timeInSeconds){
        double drivePower = 0;
        double currentTime = 0;
        while(currentTime < timeInSeconds){
            drivePower = 0.2;
            robot.leftBackMotor.setPower(drivePower);
            robot.leftFrontMotor.setPower(drivePower);
            robot.rightFrontMotor.setPower(drivePower);
            robot.rightBackMotor.setPower(drivePower);
            currentTime = System.currentTimeMillis()/1000;
        }
        drivePower = 0;
        robot.leftBackMotor.setPower(drivePower);
        robot.leftFrontMotor.setPower(drivePower);
        robot.rightFrontMotor.setPower(drivePower);
        robot.rightBackMotor.setPower(drivePower);
    }
}