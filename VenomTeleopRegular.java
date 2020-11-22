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

        robot.leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    //The run loop
    @Override
    public void loop(){
        doDrive();
        doTurn();
    }

    //The method that sets the poer to what the user inputted.
    public void doDrive(){
        double power = gamepad1.left_stick_y;


        robot.leftFrontMotor.setPower(power);
        robot.leftBackMotor.setPower(power);
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
    }
    // The method that turns the robot when the user wants the robot to turn
    // Is based off of the input that is inputted on the controller.
    public void doTurn(){
        double turnPower = gamepad1.right_stick_x;

        telemetry.addData("Turn Power", turnPower);

        if(turnPower > 0.5) {
            robot.leftFrontMotor.setPower(turnPower);
            robot.leftBackMotor.setPower(turnPower);
            robot.rightFrontMotor.setPower(-turnPower);
            robot.rightBackMotor.setPower(-turnPower);
        } else {
            robot.leftFrontMotor.setPower(-turnPower);
            robot.leftBackMotor.setPower(-turnPower);
            robot.rightFrontMotor.setPower(turnPower);
            robot.rightBackMotor.setPower(turnPower);
        }

    }

}
