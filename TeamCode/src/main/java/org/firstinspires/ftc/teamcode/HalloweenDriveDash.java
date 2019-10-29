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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.ByteLib.util.MiniPID;

@TeleOp(name="DashHalloween", group="Test")
public class HalloweenDriveDash extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftUp    = null;
    private DcMotor leftDown  = null;
    private DcMotor rightUp   = null;
    private DcMotor rightDown = null;

    private DcMotor leftArm = null;
    private DcMotor rightArm = null;

    MiniPID leftArmPID = new MiniPID(MagicNonConstants.leftArm_P,MagicNonConstants.leftArm_I,MagicNonConstants.leftArm_D);
    MiniPID rightArmPID = new MiniPID(MagicNonConstants.rightArm_P,MagicNonConstants.rightArm_I,MagicNonConstants.rightArm_D);
    int targetLeftArm = 0, targetRightArm = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("[ * ]", "Starting init...");
        telemetry.update();
        telemetry.addData("[ * ]", "DcMotor init ...");
        telemetry.update();
        leftDown   = hardwareMap.get(DcMotor.class, "l1");
        leftUp     = hardwareMap.get(DcMotor.class, "l2");
        rightUp    = hardwareMap.get(DcMotor.class, "r1");
        rightDown  = hardwareMap.get(DcMotor.class, "r2");

        leftDown.setDirection(DcMotor.Direction.FORWARD);
        leftDown.setDirection(DcMotor.Direction.FORWARD);

        rightUp.setDirection(DcMotor.Direction.REVERSE);
        rightDown.setDirection(DcMotor.Direction.REVERSE);

        leftArm    = hardwareMap.get(DcMotor.class, "leftArm");
        rightArm   = hardwareMap.get(DcMotor.class, "rightArm");
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("[ * ]", "DcMotor init [ OK ]");
        telemetry.addData("[ * ]", "DcMotor setDirection [ OK ]");
        telemetry.addData("[ ! ]", "Init: [ OK ]");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            ArcadeControl();
            int nowPosLeftArm = leftArm.getCurrentPosition();
            int nowPosRightArm = rightArm.getCurrentPosition();

            targetLeftArm = MagicNonConstants.LeftArmTARGET;
            targetRightArm = MagicNonConstants.RightArmTARGET;

            double outputLeftArm  = leftArmPID.getOutput(nowPosLeftArm,targetLeftArm);
            double outputRightArm = rightArmPID.getOutput(nowPosRightArm,targetLeftArm);

            leftArm.setPower(Range.clip(outputLeftArm, -1.0, 1.0));
            rightArm.setPower(Range.clip(outputRightArm, -1.0, 1.0));

            leftArmPID.setPID(MagicNonConstants.leftArm_P,MagicNonConstants.leftArm_I,MagicNonConstants.leftArm_D);
            rightArmPID.setPID(MagicNonConstants.rightArm_P,MagicNonConstants.rightArm_I,MagicNonConstants.rightArm_D);

            telemetry.addData("Target   : ", targetLeftArm);
            telemetry.addData("Error    : ", outputLeftArm);
            telemetry.addData("LeftPID  : ", outputLeftArm);
            telemetry.addData("RightPID : ", outputRightArm);
            telemetry.addData("Run Time : ",runtime.toString());
            telemetry.update();
        }
    }


    void run(double pow_r, double pow_l){
        leftDown.setPower(pow_l);
        leftUp.setPower(pow_l);
        rightDown.setPower(pow_r);
        rightUp.setPower(pow_r);
    }

    void ArcadeControl(){

        float yValue = gamepad1.right_stick_y * -1;
        float xValue = gamepad1.right_stick_x * -1;

        float leftPower =  yValue - xValue;
        float rightPower = yValue + xValue;

        run(Range.clip(leftPower, -1.0, 1.0),Range.clip(rightPower, -1.0, 1.0));

        telemetry.addData("Mode", "running");
        telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
        telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
        telemetry.update();
    }
}
