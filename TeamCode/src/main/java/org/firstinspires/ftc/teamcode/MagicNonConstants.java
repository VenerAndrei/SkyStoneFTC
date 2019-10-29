package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class MagicNonConstants {
    public static DcMotorSimple.Direction DIR = DcMotorSimple.Direction.FORWARD;
    public static double leftArm_P = 0.001;
    public static double leftArm_I = 0;
    public static double leftArm_D = 0;

    public static double rightArm_P = 0.001;
    public static double rightArm_I = 0;
    public static double rightArm_D = 0;

    public static int LeftArmTARGET  = 0;
    public static int RightArmTARGET = 0;

}

