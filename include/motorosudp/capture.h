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
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    bool StreamOption =0, Mearsure_Ready = 0;
    bool CreateMaskSingalAlready,MaskSingal = 0;
    size_t ChooseOBject;
    QImage BackgroundImage;
    cv::Point MaskTLPoint, MaskBRPoint = cv::Point(0,0);
    float x_robot, y_robot, z_robot, Rx, Ry, Rz;
    bool Found_Object, Found_Object_Mid = 0;


Q_SIGNALS:
    void newPixmapCaptured_Color();
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


};

#endif // CAPTURE_H


