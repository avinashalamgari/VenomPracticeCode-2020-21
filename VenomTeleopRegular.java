package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Prototype Regular TeleOp", group = "Teleop")
public class VenomTeleopRegular extends OpMode {

    HardwareTest robot = new HardwareTest();

    double horizontalPosition = robot.HORIZONTAL_HOME;
    final double HORIZONTAL_ARM_SPEED = 0.01;


    // Initializing all of the motors
    @Override
    public void init(){
        robot.leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        robot.leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        robot.rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        robot.rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");

        robot.transitionBeltLeft = hardwareMap.dcMotor.get("transitionBeltLeft");
        robot.transitionBeltRight = hardwareMap.dcMotor.get("transitionBeltRight");

        robot.shooter = hardwareMap.dcMotor.get("shooter");
        robot.intake = hardwareMap.dcMotor.get("intake");

        robot.vertical = hardwareMap.crservo.get("servo");
        robot.horizontal = hardwareMap.servo.get("servo2");

        robot.rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    //The run loop
    @Override
    public void loop(){
        doDrive();
        doTurn();
        doStrafe();
        shootRing();
        intake();
        conveyerBelt();
        servoVertical();
        servoHorizontal();
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
        else if(turnPower < -0.5) {
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

    public void shootRing(){
        double shootPower = 1;
        telemetry.addData("shoot power", shootPower);
        if(gamepad1.right_trigger > 0.2){
            robot.shooter.setPower(shootPower);
        } else{
            robot.shooter.setPower(0);
        }
    }

    public void intake(){
        double intakePower = gamepad1.left_trigger;
        boolean xPressed = gamepad1.x;
        telemetry.addData("Intake power", intakePower);
        if(intakePower > 0.2 && xPressed == false) {
            robot.intake.setPower(-1);
        } else if(intakePower > 0.2 && xPressed == true){
            robot.intake.setPower(-0.5);
        }
        else{
            robot.intake.setPower(0);
        }
    }
    public void conveyerBelt(){
        boolean yPressed = false;
        if(gamepad1.y){
            robot.transitionBeltLeft.setPower(1);
            robot.transitionBeltRight.setPower(-1);
        } else{
            robot.transitionBeltLeft.setPower(0);
            robot.transitionBeltRight.setPower(0);
        }
    }
    public void servoVertical(){
        robot.vertical.setPower(0);
        if(gamepad1.dpad_up){
            robot.vertical.setPower(1);
        } else if (gamepad1.dpad_down){
            robot.vertical.setPower(-1);
        }


        //telemetry.addData("vertical arm position",);
    }

    public void servoHorizontal(){

        if(gamepad1.dpad_left){
            horizontalPosition += HORIZONTAL_ARM_SPEED;
        } else if(gamepad1.dpad_right){
            horizontalPosition -= horizontalPosition;
        }

        horizontalPosition = Range.clip(horizontalPosition, robot.HORIZONTAL_MIN_RANGE, robot.HORIZONTAL_MAX_RANGE);

        robot.horizontal.setPosition(horizontalPosition);

    }

}
