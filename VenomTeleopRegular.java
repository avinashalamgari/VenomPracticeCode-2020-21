package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        //import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Prototype Regular TeleOp", group = "Teleop")
public class VenomTeleopRegular extends OpMode {

//    private DcMotor leftFrontMotor;
//    private DcMotor rightFrontMotor;
//    private DcMotor leftBackMotor;
//    private DcMotor rightBackMotor;

    //private Servo arm;

    HardwareTest robot = new HardwareTest();

    @Override
    public void init(){
        robot.leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        robot.leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        robot.rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        robot.rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");

        robot.leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }


    @Override
    public void loop(){
        doDrive();
        doTurn();
    }

    public void doDrive(){
        double power = gamepad1.left_stick_y;


        robot.leftFrontMotor.setPower(power);
        robot.leftBackMotor.setPower(power);
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
    }

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
