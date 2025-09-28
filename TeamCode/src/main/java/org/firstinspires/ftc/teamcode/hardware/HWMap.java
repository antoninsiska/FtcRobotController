package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

import java.util.HashMap;

public class HWMap {
    public final DcMotorEx fl, fr, bl, br;
    public HWMap(HardwareMap hw){
        fl = hw.get(DcMotorEx.class, RobotConfig.FL);
        fr = hw.get(DcMotorEx.class, RobotConfig.FR);

        bl = hw.get(DcMotorEx.class, RobotConfig.BL);
        br = hw.get(DcMotorEx.class, RobotConfig.BR);
    }
}
