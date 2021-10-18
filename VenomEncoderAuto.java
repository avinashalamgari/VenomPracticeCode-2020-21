package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "encoderauto", group = "6209")

public class VenomEncoderAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private VenomHardwareMap robot = new VenomHardwareMap();
    private VenomSensorBNO055IMU gyro = new VenomSensorBNO055IMU();


    public void runOpMode(){

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset

        forwardAuto(1, 5);
        PIDLoop(90);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    private double distToEncoder(double dist){
        return dist/(Math.PI * 3.779);
    }

    private void forwardAuto(double power, double dist){
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftBackDrive.setTargetPosition((int) dist);
        robot.leftFrontDrive.setTargetPosition((int) dist);
        robot.rightBackDrive.setTargetPosition((int) dist);
        robot.rightFrontDrive.setTargetPosition((int) dist);

        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);

        while(robot.leftBackDrive.isBusy() || robot.leftFrontDrive.isBusy() || robot.rightBackDrive.isBusy() || robot.rightFrontDrive.isBusy()){

        }
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
    }

    private void turn(double power) {

        robot.leftBackDrive.setPower(power);
        robot.leftFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(-power);
        robot.rightFrontDrive.setPower(-power);

    }

    private double angleToDistance(double angle) {
        /*double ratio = angle/360;
        return (2 * Math.PI * 8.5) * ratio;
*/
        /*double cmfe = Math.atan((11.27 / 14.549) * Math.tan(angle));
        double derparametricx = -11.27 * Math.sin(cmfe);
        double derparametricy = 14.549 * Math.cos(cmfe);*/
        return 0.0;
    }

    public void PIDLoop(double target){
        double current = gyro.getHeading();
        double integral = 0;
        double prevError = 0;
        double prevTime = 0;
        double turnPower = Integer.MAX_VALUE;
        double currTime = 0;

        double error = target - current;
        double kP = 0.6 / 90;
        double kI = 0.012;
        double kD = 0.02 / 90;

        while (Math.abs(error) >= 1 && turnPower < 0)  {
            current = gyro.getHeading();
            error = Math.abs(target - current);
            double proportional = error * kP;
            currTime = getRuntime()/Math.pow(10,9);
            integral += error * (currTime - prevTime) * kI;
            double derivative = (error - prevError) / (currTime - prevTime) * kD;
            turnPower = proportional + integral + derivative;
            turn(turnPower);
            prevTime = getRuntime();
            prevError = error;
        }

    }

    }





