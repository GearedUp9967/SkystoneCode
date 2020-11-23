package org.firstinspires.ftc.teamcode;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class StackVision extends OpenCvPipeline {
    public int width;
    public int height;
    private Mat hsv = new Mat();
    private Mat orange = new Mat();
    private Mat display = new Mat();

    private Scalar red = new Scalar(255,0,0);
    private Scalar blue = new Scalar(0,0,255);
    private Scalar purple = new Scalar(255,0,255);
    private Scalar green = new Scalar(0,255,0);
    private int num=0;
    private int line;
    private int blockwid;


    public int numPixels=0;



    //zero is the default if it doesnt know
    private int number=0;

    private boolean redSide=true;
    private boolean right=false;


    public StackVision(boolean redside) {
        redSide=redside;
    }

    @Override
    public Mat processFrame(Mat rgba) {
        width = rgba.width();
        height = rgba.height();
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);

        Core.inRange(hsv, new Scalar(6, 100, 100), new Scalar(17, 255, 255), orange);


        double top=.538*height;
        double bottom=.643*height;
        double boxHeight=bottom-top;
        double diskWidth=.203*width;
        double cutOffOffset=diskWidth-(width-.886*width);

        double xcoord=0;
        if(right && redSide)
        {
            xcoord=0;
        }
        else{
            xcoord=width-diskWidth;
        }



        Rect diskBox=new Rect((int) xcoord,(int) top,(int) diskWidth,(int) boxHeight);




        Imgproc.cvtColor(orange,display,Imgproc.COLOR_GRAY2RGB,3);



        Imgproc.rectangle(display,diskBox,green);

        return display;
    }

    /*
    private int scanPixels(boolean rightToLeft)
    {

        int start=0;
        int inc=1;
        if(!rightToLeft)
        {
            start=width-1;
            inc=-1;
        }


        boolean startedCounting=false;
        int blackGap=0;
        int pixels=0;
        int i=start;
        Position position= Position.UNKNOWN;
        while(0<=i && i<width) {
            if(Core.countNonZero(yellow.submat(new Rect(i,line,1,1)))==0) {
                if(startedCounting)
                {
                    blackGap+=1;
                    if(blackGap>10)
                    {
                        break;
                    }
                }
            }
            else
            {
                blackGap=0;
                startedCounting=true;
                pixels+=1;
            }

            i+=inc;
        }


        return pixels;
    }

    */



}
