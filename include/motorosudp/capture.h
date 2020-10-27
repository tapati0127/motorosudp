#ifndef CAPTURE_H
#define CAPTURE_H

#include <QPixmap>
#include <QImage>
#include <QThread>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#define ID_CAMERA 0

class Capture : public QThread  //MyVideoCapture
{
    Q_OBJECT
public:
    Capture(QObject *parent = nullptr); //MyVideoCapture
    QPixmap pixmap_Color() const
    {
        return mPixmap_Color ;
    }
    void detectOject_findContour(cv::Mat m_Frame, cv::Mat Frame_Draw, std::vector < std::vector<cv::Point> > blobs,
                                 size_t &numberblob,rs2::depth_frame depth, bool Mearsure_Ready,
                                 float &x_robot, float &y_robot, float &z_robot, bool &Found_Object,bool &Found_Object_Mid,
                                 float &Rx, float &Ry, float &Rz);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    bool StreamOption =0, Mearsure_Ready = 0;
    bool CreateMaskSingalAlready,MaskSingal = 0;
    size_t ChooseOBject;
    QImage BackgroundImage;
    cv::Point MaskTLPoint, MaskBRPoint = cv::Point(0,0);
    float x_robot, y_robot, z_robot, Rx, Ry, Rz = 0;
    bool Found_Object, Found_Object_Mid = 0;


Q_SIGNALS:
    void newPixmapCaptured_Color();
    void foundObject();
protected:
    void run() override;
private:
    QPixmap mPixmap_Color;
    cv::Mat mFrame_Color ;
    cv::Mat mFrame_Depth ;
    cv::Mat BGSubtraction;
    cv::VideoCapture mVideoCap;
    QImage cvMatToQImage(const cv::Mat &inMat);
    QPixmap cvMatToQPixmap(const cv::Mat &inMat);
    cv::Mat QImageToMat(QImage image);
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    rs2::config cfg;
    float z_robot_now, z_robot_1, z_robot_2, z_robot_3 = 0;
    int cnt;
    double sum;
    int time;


};

#endif // CAPTURE_H


