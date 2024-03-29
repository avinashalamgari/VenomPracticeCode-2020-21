/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Linear4cTeleop", group="TeleOp")
@Disabled
public class VenomLinearTeleop extends LinearOpMode {

    /* Declare OpMode members. */

    VenomHardwareMap robot = new VenomHardwareMap();

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("WaitForStart", "Waiting For Start");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.left_stick_y != 0){
                forward(gamepad1.left_stick_y);
            }

            if(gamepad1.left_stick_x > 0){
                right(gamepad1.left_stick_x);
            } else if(gamepad1.left_stick_x < 0){
                left(gamepad1.left_stick_x);
            }

            if (gamepad1.dpad_up == true) {
                lift(.30);
            }
                else if(gamepad1.dpad_down == true){
                    lift(-.30);
            }
        }
    }

    public void forward(double power) {
        robot.rightFrontDrive.setPower(power);
        robot.leftFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
    }

    public void left(double power) {
        robot.rightFrontDrive.setPower(power);
        robot.leftFrontDrive.setPower(-power);
        robot.rightBackDrive.setPower(power);
        robot.leftBackDrive.setPower(-power);
    }
    public void right(double power) {
        robot.rightFrontDrive.setPower(-power);
        robot.leftFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(-power);
        robot.leftBackDrive.setPower(power);
    }

    public void lift(double liftPower){
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int currentPosition = robot.leftArm.getCurrentPosition();
        robot.leftArm.setTargetPosition(currentPosition + 30);
        robot.rightArm.setTargetPosition(currentPosition + 30);
        while (robot.leftArm.isBusy()){
            robot.leftArm.setPower(liftPower);
            robot.rightArm.setPower(liftPower);
        }

    }


}
