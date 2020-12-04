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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;




public class Rick_HardwareMap
{
    public DcMotor RightFrontMotor = null;
    public DcMotor RightBackMotor = null;
    public DcMotor LeftFrontMotor = null;
    public DcMotor LeftBackMotor = null;
    public DcMotor LeftBrush = null;
    public DcMotor RightBrush = null;

    HardwareMap Rick_hwm = null;
    
    public void init(HardwareMap ahwMap) {
    Rick_hwm=ahwMap;

    RightFrontMotor = Rick_hwm.get(DcMotor.class, "RightFrontMotor");
    RightBackMotor = Rick_hwm.get(DcMotor.class, "RightBackMotor");
    LeftFrontMotor = Rick_hwm.get(DcMotor.class, "LeftFrontMotor");
    LeftBackMotor = Rick_hwm.get(DcMotor.class, "LeftBackMotor");
    LeftBrush = Rick_hwm.get(DcMotor.class, "LeftBrush");
    RightBrush = Rick_hwm.get(DcMotor.class, "RightBrush");

    RightBackMotor.setDirection(REVERSE);
    RightFrontMotor.setDirection(REVERSE);
    LeftFrontMotor.setDirection(FORWARD);
    LeftBackMotor.setDirection(FORWARD);
    RightBrush.setDirection(FORWARD);
    LeftBrush.setDirection(REVERSE);

    RightBackMotor.setPower(0);
    RightFrontMotor.setPower(0);
    LeftFrontMotor.setPower(0);
    LeftBackMotor.setPower(0);
    RightBrush.setPower(0);
    LeftBrush.setPower(0);
    }



}
