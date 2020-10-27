#include "../include/motorosudp/capture.h"
#include <QDebug>
#include<time.h>

Capture::Capture(QObject *parent)
    :QThread { parent }
    //, mVideoCap{ ID_CAMERA}

{   

}
cv::Mat CreateMask (cv::Point TL, cv::Point BR ,QImage BackgroundImage)
{
    cv::Mat Mask(BackgroundImage.height(),BackgroundImage.width(), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::rectangle(Mask, BR, TL, cv::Scalar(255, 255, 255),-1);
    return Mask;
}

cv::Mat Background_Subtraction(cv::Mat Frame, cv::Mat BackgroundMat, cv::Mat Mask, cv::Point TL, cv::Point BR)
{
    cv::Mat FrameGray, BGGray, MaskGray, Subtraction;
    cv::cvtColor(Frame, FrameGray, cv::COLOR_BGR2GRAY );
    cv::cvtColor(BackgroundMat, BGGray, cv::COLOR_BGR2GRAY );
    cv::cvtColor(Mask, MaskGray, cv::COLOR_BGR2GRAY );
    cv::GaussianBlur(FrameGray, FrameGray, cv::Size(5, 5), 0 );
    cv::GaussianBlur(BGGray, BGGray, cv::Size(5, 5), 0 );

    cv::absdiff(FrameGray,BGGray,Subtraction) ;
    Subtraction = Subtraction & MaskGray ;


    int sum,cnt = 0;
    for (int i = TL.x; i < BR.x; i++)
    {
        for (int j = TL.y; j < BR.y; j++)
        {
            if ( ((int)Subtraction.at<uchar>(j,i) <= 255)&& ((int)Subtraction.at<uchar>(j,i) >= 0))
            {cnt++;
            sum += (int)Subtraction.at<uchar>(j,i) ;}
        }
    }

    float threshold = sum/cnt ;

    //std::cout << "threshold = " << threshold << std::endl;
    if (threshold <20)  threshold = 20;


    //cv::Mat AfThreshold = Subtraction(cv::Rect(TL,BR));
    cv::threshold(Subtraction,Subtraction,threshold,255,0);

    return Subtraction;

}

void labelBlobs(const cv::Mat gray, std::vector < std::vector<cv::Point> > &blobs)
{

    blobs.clear();

    // Using labels from 2+ for each blob
    cv::Mat label_image;
    cv::Mat binary;

    cv::dilate(gray, gray, cv::Mat(),cv::Point(-1,-1) );
    cv::erode(gray, gray, cv::Mat(),cv::Point(-1,-1));


    gray.copyTo(label_image);


    int label_count = 2; // starts at 2 because 0,1 are used already


    for(int y=0; y < gray.rows; y++) {
        for(int x=0; x < gray.cols; x++) {
            if((int)label_image.at<uchar>(y,x) != 255) {

                continue;
            }

            cv::Rect rect;
            cv::floodFill(label_image, cv::Point(x,y), cv::Scalar(label_count), &rect, cv::Scalar(0), cv::Scalar(0), 4);

            std::vector<cv::Point>  blob;

            for(int i=rect.y; i < (rect.y+rect.height); i++) {
                for(int j=rect.x; j < (rect.x+rect.width); j++) {
                    if((int)label_image.at<uchar>(i,j) != label_count) {
                        continue;
                    }

                    blob.push_back(cv::Point(j,i));
                }
            }
            if ((blob.size() > 5000)&&(blob.size() <8000))
                blobs.push_back(blob);

            label_count++;
        }
    }

}

float FindDistance (rs2::depth_frame depth_frame,cv::Point center,float &x,float &y, float &z)
{

    float dis = depth_frame.get_distance(int(center.x),int(center.y));
    float center_pixel[2],center_point[3] ;
    center_pixel[0] = int(center.x); center_pixel[1] = int(center.y);

    rs2_intrinsics intr = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    rs2_deproject_pixel_to_point(center_point, &intr, center_pixel, dis);
    x = center_point[0] ; y = center_point[1] ; z = center_point[2];
    return dis;

}

void Mearsure_Size (rs2::depth_frame depth_frame,cv::Point2f tltr,cv::Point2f blbr,cv::Point2f tlbl,cv::Point2f trbr,
                     float &width , float &height )
{
    float tltr_pixel[2], blbr_pixel[2], tlbl_pixel[2], trbr_pixel[2];
    tltr_pixel[0] = int(tltr.x); tltr_pixel[1] =  int(tltr.y);
    blbr_pixel[0] = int(blbr.x); blbr_pixel[1] =  int(blbr.y);

    tlbl_pixel[0] = int(tlbl.x) + 1 ; tlbl_pixel[1] =  int(tlbl.y);
    trbr_pixel[0] = int(trbr.x) - 1; trbr_pixel[1] =  int(trbr.y);

    float tltr_point[3],blbr_point[3] ;
    float tlbl_point[3],trbr_point[3] ;

    rs2_intrinsics intr = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    auto tltr_dis = depth_frame.get_distance(tltr_pixel[0],tltr_pixel[1]);
    auto blbr_dis = depth_frame.get_distance( blbr_pixel[0],blbr_pixel[1]);


    rs2_deproject_pixel_to_point(tltr_point, &intr, tltr_pixel, tltr_dis);
    rs2_deproject_pixel_to_point(blbr_point, &intr, blbr_pixel, blbr_dis);

     width = sqrt(pow(tltr_point[0] - blbr_point[0], 2) +
            pow(tltr_point[1] - blbr_point[1], 2) +
            pow(tltr_point[2] - blbr_point[2], 2));


    auto tlbl_dis = depth_frame.get_distance(tlbl_pixel[0],tlbl_pixel[1]);
    auto trbr_dis = depth_frame.get_distance( trbr_pixel[0],trbr_pixel[1]);


    rs2_deproject_pixel_to_point(tlbl_point, &intr, tlbl_pixel, tlbl_dis);
    rs2_deproject_pixel_to_point(trbr_point, &intr, trbr_pixel, trbr_dis);

    height = sqrt(pow(tlbl_point[0] - trbr_point[0], 2) +
            pow(tlbl_point[1] - trbr_point[1], 2) +
            pow(tlbl_point[2] - trbr_point[2], 2));

}

void Tranfer_CorRobot(float x_cam, float y_cam, float z_cam,
                      float &x_robot, float &y_robot, float &z_robot)
{
    //Tranfer Matrix
    float A[4][4] =
      {
        { 0.0349, 0.999, 0,  238 },
        { 0.999, -0.034, 0, -303 },
        {     0,     0, -1, 145.7},
        {     0,     0,  0,    1 },
      };
     // Position of Object in Camera Coordinate
    float B[4][1]=
      {
        {x_cam},
        {y_cam},
        {z_cam},
        {1}
      };
    // Multiply 2 Matrix : C = A*B
      float C[4][1] ;
      int i,j,k;
      for(i=0;i<4;i++)
        for(j=0;j<1;j++)
        {
          C[i][j]=0;
          for(k=0;k<4;k++)
          {
            C[i][j]+=A[i][k]*B[k][j];
          }
        }
      //Object Position in Robot Coordinate
      x_robot = C[0][0];
      y_robot = C[1][0];
      z_robot = C[2][0];
}
void Capture::detectOject_findContour(cv::Mat m_Frame, cv::Mat Frame_Draw, std::vector < std::vector<cv::Point> > blobs,
                             size_t &numberblob,rs2::depth_frame depth, bool Mearsure_Ready,
                             float &x_robot, float &y_robot, float &z_robot, bool &Found_Object,bool &Found_Object_Mid,
                             float &Rx, float &Ry, float &Rz)
{
    cv::RotatedRect marker;
    cv::Mat boxPts,mean;
    cv::Mat edged;

    cv::Mat Mask(m_Frame.rows,m_Frame.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat MaskGray;
    cv::cvtColor(Mask,MaskGray,cv::COLOR_BGR2GRAY);

    //#########################################//
    if (!blobs.empty())
    {
        if (numberblob >= blobs.size()) {numberblob = 0;}


        for(size_t i=0; i < blobs.at(numberblob).size(); i++)
        {

            MaskGray.at<uchar>(blobs.at(numberblob).at(i).y,blobs.at(numberblob).at(i).x) = 255 ;

        }


    cv::Canny( MaskGray, edged, 35, 125 );
    cv::dilate(edged, edged, cv::Mat(),cv::Point(-1,-1) );
    cv::erode(edged, edged, cv::Mat(),cv::Point(-1,-1));

    std::vector<std::vector<cv::Point> > cnts;
    cv::findContours( edged, cnts, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

    //Find largest countour
    int largest_area=0;
    double largest_contour_index=0;
    for( size_t i = 0; i< cnts.size(); i++ ) // iterate through each contour.
        {
            double area = cv::contourArea( cnts[i] );  //  Find the area of contour

            if (area > largest_area)
            {
              largest_area = area;
              largest_contour_index = i;               //Store the index of largest contour

            }
        }
    Frame_Draw = m_Frame;


    //cv::drawContours( Frame_Draw, cnts, largest_contour_index, cv::Scalar(0, 0, 255), 2 );
    marker = cv::minAreaRect(cnts[largest_contour_index]);
    cv::boxPoints(marker,boxPts);
    cv::Point2f rect_points[4];
    marker.points(rect_points);
    for(int i = 0; i <4; i++)
        cv::line(Frame_Draw,rect_points[i],rect_points[(i+1)%4],cv::Scalar(0, 0, 255) );

    //in bottom-left,top-left, top-right, bottom-right
    cv::reduce(boxPts, mean, 0, cv::REDUCE_AVG);
    cv::Point2f center(mean.at<float>(0,0), mean.at<float>(0,1));



    // Calculate Theta
    cv::Point2f tl = cv::Point2f(boxPts.at<float>(1,0),boxPts.at<float>(1,1));
    cv::Point2f tr = cv::Point2f(boxPts.at<float>(2,0),boxPts.at<float>(2,1));
    cv::Point2f bl = cv::Point2f(boxPts.at<float>(0,0),boxPts.at<float>(0,1));
    cv::Point2f A =tl ,B = tl;
    double theta;
    if (((tl.x - tr.x)*(tl.x - tr.x)* + (tl.y - tr.y)*(tl.y - tr.y)) >
          ((tl.x - bl.x)*(tl.x - bl.x)* + (tl.y - bl.y)*(tl.y - bl.y))   )
        B = tr;
    else if (((tl.x - tr.x)*(tl.x - tr.x)* + (tl.y - tr.y)*(tl.y - tr.y)) <
             ((tl.x - bl.x)*(tl.x - bl.x)* + (tl.y - bl.y)*(tl.y - bl.y))   )
        B = bl;

    if (A != B) theta = std::atan(-(A.y - B.y)/(A.x - B.x))*180/3.14;
    else theta =0;
    //Calculate Rx, Ry, Rz
    Rx = 180;
    Ry = 0;
    Rz =theta;
    //between the top-left and top-right coordinates, followed by
    //the midpoint between bottom-left and bottom-right coordinates
    cv::Point2f tltr =  (cv::Point2f(boxPts.at<float>(2,0),boxPts.at<float>(2,1))+
                         cv::Point2f(boxPts.at<float>(1,0),boxPts.at<float>(1,1)))*.5;
    cv::Point2f blbr =  (cv::Point2f(boxPts.at<float>(0,0),boxPts.at<float>(0,1))+
                         cv::Point2f(boxPts.at<float>(3,0),boxPts.at<float>(3,1)))*.5;
    //compute the midpoint between the top-left and bottom-left points,
    //followed by the midpoint between the top-right and bottom-right
    cv::Point2f tlbl =  (cv::Point2f(boxPts.at<float>(0,0),boxPts.at<float>(0,1))+
                     cv::Point2f(boxPts.at<float>(1,0),boxPts.at<float>(1,1)))*.5;
    cv::Point2f trbr =  (cv::Point2f(boxPts.at<float>(2,0),boxPts.at<float>(2,1))+
                     cv::Point2f(boxPts.at<float>(3,0),boxPts.at<float>(3,1)))*.5;

    //Calculate Size
    if (Mearsure_Ready == 1)
    {
        float width, height;
        Mearsure_Size(depth,tltr,blbr,tlbl,trbr,width,height);

        char text_height[200] , text_width[200];
        sprintf(text_height,"%.1f cm",height*100);
        sprintf(text_width,"%.1f cm",width*100);

        cv::putText(Frame_Draw, text_height,
            cv::Point2f(tltr.x - 15, tltr.y - 10), cv::FONT_HERSHEY_SIMPLEX,
            0.65, cv::Scalar(255, 255, 255), 2);
        cv::putText(Frame_Draw, text_width,
            cv::Point2f(trbr.x + 10, trbr.y), cv::FONT_HERSHEY_SIMPLEX,
            0.65, cv::Scalar(255, 255, 255), 2);
    }
    else
    {
    float x_cam,y_cam,z_cam;
    float dis = FindDistance(depth,center, x_cam,y_cam,z_cam );

    Tranfer_CorRobot(x_cam*1000, y_cam*1000, z_cam*1000, x_robot,y_robot,z_robot);
    //Set signal for Robot
    if(y_robot >= -255) {
      bool temp = 1;
      if(temp!=Found_Object_Mid)
        Q_EMIT foundObject();
      Found_Object_Mid = temp;
    }
    else Found_Object_Mid = 0;

    //String
    char text[200];

    sprintf(text,"(%.2f %.2f %.2f %.f)",x_cam*1000, y_cam*1000, z_cam*1000, theta);

    cv::circle(Frame_Draw, center, 3, cv::Scalar(0, 255, 0), -1);


    cv::putText(Frame_Draw, text,
        cv::Point(Frame_Draw.size[1] - 250, Frame_Draw.size[0] - 20), cv::FONT_HERSHEY_SIMPLEX,
        0.65, cv::Scalar(0, 255, 0), 2);
    //std::cout << this->x_robot << " " << this->y_robot << " " << this->z_robot << std::endl;
    }
    }
}

void CalVelocity(time_t &start, time_t &end, float y_robot, float &Velocity)
{
    float start_position = 100, end_poition = 200;
    if (y_robot < start_position)
    {start = 0; end = 0; Velocity = 0;}
    if ((y_robot > start_position)&&(start==0))
        time(&start);
    if ((y_robot > end_poition)&&(end == 0 )&&(Velocity == 0))
    {
        time(&end);
        Velocity = (end_poition-start_position)/(end-start);
    }

}


void Capture::run()
{
    // Start streaming with default recommended configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    pipe.start(cfg);
    CreateMaskSingalAlready = 0;
    rs2::align align_to_color(RS2_STREAM_COLOR);

        while (true)
        {
            //std::cout << double(clock()-time)/CLOCKS_PER_SEC << std::endl;
            time = clock();

            rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

            data = align_to_color.process(data);

            rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
            rs2::frame color = data.get_color_frame();

            //Another depth_Frame
            rs2::depth_frame depth_frame = data.get_depth_frame();


            // Query frame size (width and height)
            const int w_depth = depth.as<rs2::video_frame>().get_width();
            const int h_depth = depth.as<rs2::video_frame>().get_height();

            const int w_color = color.as<rs2::video_frame>().get_width();
            const int h_color = color.as<rs2::video_frame>().get_height();


            // Create OpenCV matrix of size (w,h) from the colorized depth data
            mFrame_Depth = cv::Mat (cv::Size(w_depth, h_depth), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
            mFrame_Color = cv::Mat (cv::Size(w_color, h_color), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat mFrame_Depth_another = cv::Mat (cv::Size(w_depth, h_depth), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
            mFrame_Depth_another.convertTo(mFrame_Depth_another, CV_8UC1, 15 / 256.0);

            //resize(mFrame_Color, mFrame_Color,cv::Size(480, 360), 0, 0, cv::INTER_CUBIC);
           // resize(mFrame_Depth, mFrame_Depth,cv::Size(480, 360), 0, 0, cv::INTER_CUBIC);
            /*cv::Mat m_Frame;
            mVideoCap >> m_Frame;
            resize(m_Frame, mFrame_Color,cv::Size(480, 360), 0, 0, cv::INTER_CUBIC);
            resize(m_Frame, mFrame_Depth,cv::Size(480, 360), 0, 0, cv::INTER_CUBIC);*/


            //Step 1: Background Subtraction
            cv::Mat BackgroundMat = QImageToMat(BackgroundImage);
            cv::Mat Mask;
            if (!BackgroundMat.empty() && MaskSingal==1)
            {
                Mask = CreateMask(MaskTLPoint,MaskBRPoint,BackgroundImage);


                CreateMaskSingalAlready = 1;

            }

            if (!BackgroundMat.empty() && CreateMaskSingalAlready ==1)
            {
                //std::cout << "w : \n" << 1 << std::endl;
                BGSubtraction = Background_Subtraction(mFrame_Color,BackgroundMat,Mask,MaskTLPoint,MaskBRPoint);
                //Step 2: Connected component labeling
                std::vector<std::vector<cv::Point> > blobs;
                //std::cout << "w : \n" << BGSubtraction.at<uchar>(180,240) << std::endl;
                //std::cout << "h : \n" << BGSubtraction.at<int>(180,240) << std::endl;
                labelBlobs(BGSubtraction,blobs);
                //Step 3: Find Contour
                cv::Mat Frame_Draw;
                detectOject_findContour(mFrame_Color,Frame_Draw, blobs, ChooseOBject,depth_frame,Mearsure_Ready,
                                        x_robot, y_robot, z_robot, Found_Object, Found_Object_Mid, Rx, Ry, Rz);

                //Calculate z_robot
                /*if (blobs.empty()) {cnt = 0 ;sum = 0 ;z_robot_now = 0 ; z_robot_1 = 0 ; z_robot_2 = 0 ; z_robot_3 = 0;}
                else
                {
                  //cnt++;
                  //sum += z_robot_now;
                  //z_robot = sum/cnt;
                  z_robot = 0.8*z_robot_now + 0.1*z_robot_1 + 0.05*z_robot_2 + 0.05*z_robot_3;
                  z_robot_3 = z_robot_2;
                  z_robot_2 = z_robot_1;
                  z_robot_1 = z_robot;
                }*/

            }
            if (!mFrame_Color.empty() && !mFrame_Depth.empty())
            {
                if(StreamOption == 0)
                    mPixmap_Color = cvMatToQPixmap(mFrame_Color);
                else
                    mPixmap_Color = cvMatToQPixmap(mFrame_Depth);
                //Test
                /*cv::Mat Blue(3,3, CV_8UC3, cv::Scalar(255,0,0));
                cv::Mat Green(3,3, CV_8UC3, cv::Scalar(0,255,0));
                cv::Mat White(3,3,CV_8UC3, cv::Scalar(255,255,255));
                cv::Mat Testing(3,3, CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

                cv::Mat BlueGray, GreenGray,WhiteGray, Sub, Testing_Gray;
                cv::cvtColor(Blue,BlueGray,cv::COLOR_BGR2GRAY);
                cv::cvtColor(Green,GreenGray,cv::COLOR_BGR2GRAY);
                cv::cvtColor(White,WhiteGray,cv::COLOR_BGR2GRAY);
                cv::cvtColor(Testing,Testing_Gray,cv::COLOR_BGR2GRAY);
                Testing_Gray.at<uchar>(2,2) = 65;
                std::cout<<"B = "<< BlueGray << std::endl;
                std::cout<<"T = "<< Testing_Gray << std::endl;
                int t;
                if(Testing_Gray.at<uchar>(2,2)==(uchar)65) t = 6;
                std::cout<<"T = "<< t << std::endl;

                Sub = BlueGray&WhiteGray;

                std::cout<<"Sub = "<< Sub << std::endl;
                cv::Mat Thres;
                cv::threshold(Sub,Thres,25,255,0);
                std::cout<<"Thres = "<< Thres << std::endl;*/

                Q_EMIT newPixmapCaptured_Color();
            }



    }


}

QImage Capture:: cvMatToQImage( const cv::Mat &inMat )  //MyVideoCapture
   {
      switch ( inMat.type() )
      {
         // 8-bit, 4 channel
         case CV_8UC4:
         {
            QImage image( inMat.data,
                          inMat.cols, inMat.rows,
                          static_cast<int>(inMat.step),
                          QImage::Format_ARGB32 );

            return image;
         }

         // 8-bit, 3 channel
         case CV_8UC3:
         {
            QImage image( inMat.data,
                          inMat.cols, inMat.rows,
                          static_cast<int>(inMat.step),
                          QImage::Format_RGB888 );

            return image.rgbSwapped();
         }

         // 8-bit, 1 channel
         case CV_8UC1:
         {
#if QT_VERSION >= QT_VERSION_CHECK(5, 5, 0)
            QImage image( inMat.data,
                          inMat.cols, inMat.rows,
                          static_cast<int>(inMat.step),
                          QImage::Format_Grayscale8 );
#else
            static QVector<QRgb>  sColorTable;

            // only create our color table the first time
            if ( sColorTable.isEmpty() )
            {
               sColorTable.resize( 256 );

               for ( int i = 0; i < 256; ++i )
               {
                  sColorTable[i] = qRgb( i, i, i );
               }
            }

            QImage image( inMat.data,
                          inMat.cols, inMat.rows,
                          static_cast<int>(inMat.step),
                          QImage::Format_Indexed8 );

            image.setColorTable( sColorTable );
#endif

            return image;
         }

         default:
            qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
            break;
      }

      return QImage();
   }

QPixmap Capture:: cvMatToQPixmap( const cv::Mat &inMat )  //MyVideoCapture
   {
      return QPixmap::fromImage( cvMatToQImage( inMat ) );
   }

cv::Mat Capture::QImageToMat(QImage image)
{
    cv::Mat mat;
    switch (image.format())
    {
    case QImage::Format_ARGB32:
    case QImage::Format_RGB32:
    case QImage::Format_ARGB32_Premultiplied:
        mat = cv::Mat(image.height(), image.width(), CV_8UC4, (void*)image.constBits(), image.bytesPerLine());
        break;
    case QImage::Format_RGB888:
        mat = cv::Mat(image.height(), image.width(), CV_8UC3, (void*)image.constBits(), image.bytesPerLine());
        cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
        break;
    /*case QImage::Format_Grayscale8:
        mat = cv::Mat(image.height(), image.width(), CV_8UC1, (void*)image.constBits(), image.bytesPerLine());
        break;*/
    }
    return mat;
}
