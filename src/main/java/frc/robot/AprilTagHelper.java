package frc.robot;

import frc.robot.subsystems.LimelightHelpers;

public class AprilTagHelper {
    public static int tidFromLimelight()
    {
        return (int)LimelightHelpers.getFiducialID("");
    }

    public static int reefAngleFromTid(int tid)
    {
        if (tid == 9 || tid == 22)
            return 120;
        else if (tid == 8 || tid == 17)
            return 60;
        else if (tid == 7 || tid == 18)
            return  0;
        else if (tid == 6 || tid == 19)
            return 300;
        else if (tid == 11 || tid == 20)
            return 240;
        else if (tid == 21 || tid == 10)
            return 180;
        else
            return -1;
        
        // For Off Season tic-tac-toe board
        
        // if (tid == 1 || tid == 2 || tid == 3 || tid == 4)
        //     return 0;
        // else
        //     return -1;
    }

}
