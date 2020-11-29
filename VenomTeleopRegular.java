package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Prototype Regular TeleOp", group = "Teleop")
public class VenomTeleopRegular extends OpMode {

    HardwareTest robot = new HardwareTest();


    // Initializing all of the motors
    @Override
    public void init(){
        robot.leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        robot.leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        robot.rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        robot.rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");

        robot.rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    //The run loop
    @Override
    public void loop(){
        doDrive();
        doTurn();
        doStrafe();

    }

    //The method that sets the poer to what the user inputted.
    public void doDrive(){
        double drivePower = gamepad1.left_stick_y;
        telemetry.addData("Drive Power", drivePower);


        robot.leftFrontMotor.setPower(drivePower);
        robot.leftBackMotor.setPower(-drivePower);
        robot.rightFrontMotor.setPower(drivePower);
        robot.rightBackMotor.setPower(-drivePower);
    }
    // The method that turns the robot when the user wants the robot to turn
    // Is based off of the input that is inputted on the controller.
    public void doTurn(){
        double turnPower = gamepad1.right_stick_x;

        telemetry.addData("Turn Power", turnPower);

        if(turnPower > 0.5) {
            robot.leftFrontMotor.setPower(-turnPower);
            robot.leftBackMotor.setPower(turnPower);
            robot.rightFrontMotor.setPower(turnPower);
            robot.rightBackMotor.setPower(-turnPower);
        }
        if(turnPower < -0.5) {
            robot.leftFrontMotor.setPower(-turnPower);
            robot.leftBackMotor.setPower(turnPower);
            robot.rightFrontMotor.setPower(turnPower);
            robot.rightBackMotor.setPower(-turnPower);
        }

    }

    public void doStrafe(){
        double strafePower = gamepad1.left_stick_x;

        telemetry.addData("strafe Power", strafePower);

        if(strafePower > 0.5) {
            robot.leftFrontMotor.setPower(-strafePower);
            robot.leftBackMotor.setPower(-strafePower);
            robot.rightFrontMotor.setPower(strafePower);
            robot.rightBackMotor.setPower(strafePower);
        }
        if(strafePower < -0.5) {
            robot.leftFrontMotor.setPower(-strafePower);
            robot.leftBackMotor.setPower(-strafePower);
            robot.rightFrontMotor.setPower(strafePower);
            robot.rightBackMotor.setPower(strafePower);
        }
    }


}
