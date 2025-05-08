package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitCams.OrbitWebcam.TempVision.threshold;

import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitCams.OrbitWebcam.TempVision.threshold.enums.PropColorEnum;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitCams.OrbitWebcam.TempVision.threshold.enums.PropPosEnum;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitCams.OrbitWebcam.TempVision.threshold.enums.YellowPixelPosEnum;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

import java.util.HashSet;

public class ElementDetectBox {
    public static PropPosEnum propPos = PropPosEnum.NONE;

    public YellowPixelPosEnum place;
    public PropColorEnum elementColor = PropColorEnum.YELLOW;
    public Rect elementBox;
    public double box;
    public double averagedBox = 0;

    public ElementDetectBox(YellowPixelPosEnum place, Rect rect) {
        this.place = place;
        this.elementBox = rect;
    }
    public void boxAverageUpdate(Mat mat) {
        try {
            this.box = Core.sumElems(mat.submat(this.elementBox)).val[0];
        } catch (Exception e) {
            this.box = 0;
        }

        if ( (propPos == PropPosEnum.LEFT && this.place == YellowPixelPosEnum.MISSLEFT)  ||
             (propPos == PropPosEnum.RIGHT && this.place == YellowPixelPosEnum.MISSRIGHT)   ) {
            this.box = 0;
        }

        this.averagedBox = this.box / this.elementBox.area() / 255;
    }

    public static ElementDetectBox max(HashSet<ElementDetectBox> boxList) {
        ElementDetectBox highestScoreBox = null;

        for (ElementDetectBox eBox: boxList) {
            if (highestScoreBox == null) {
                highestScoreBox = eBox;
            }
            else {
                if (eBox.averagedBox > highestScoreBox.averagedBox ||
                        (eBox.averagedBox == highestScoreBox.averagedBox) && (eBox.box > highestScoreBox.box)) {
                    highestScoreBox = eBox;
                }
            }
        }

        return highestScoreBox;
    }
}
