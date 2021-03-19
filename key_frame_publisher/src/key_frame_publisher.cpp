// #include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <key_frame_publisher/boolStamped.h>


using namespace std; 
ros::Publisher image_pub, depth_pub, kf_pub, cam_info_pub;
bool mm_to_meters;
int kf_rate;
sensor_msgs::CameraInfo camInfo;
double blurrThreshold;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

//TODO add contact here
bool isKeyFrame( uint frame, double focusMeasure)
{
    if(kf_rate!=0 && frame%kf_rate == 0 && focusMeasure >= blurrThreshold)
        return 1;
    else
        return 0;
}

// OpenCV port of 'LAPV' algorithm (Pech2000)
double varianceOfLaplacian(const cv::Mat& src)
{
    cv::Mat lap;
    cv::Laplacian(src, lap, CV_64F);

    cv::Scalar mu, sigma;
    cv::meanStdDev(lap, mu, sigma);

    double focusMeasure = sigma.val[0]*sigma.val[0];
    return focusMeasure;
}

void imageDepthCb(const sensor_msgs::ImageConstPtr &img_msg, 
                  const sensor_msgs::ImageConstPtr &depth_msg)
{    
    static uint frame=0;
    cv_bridge::CvImagePtr rgb, depth;
    try
    {
        rgb = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge RGB exception: %s", e.what());
        return;
    }
    
    cv::Mat mono;
    if (rgb->image.channels() == 3)
    {
        cvtColor(rgb->image, mono, cv::COLOR_BGR2GRAY);
    }
    else
    {
        mono = rgb->image;
    }



    double focusMeasure = varianceOfLaplacian(mono);

    try
    {
       depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge DEPTH exception: %s", e.what());
        return;
    }
    if (mm_to_meters)
        depth->image *= 0.001;
    
    ros::Time now = ros::Time::now();      
    
    key_frame_publisher::boolStamped bool_msg;    
    bool_msg.indicator.data=isKeyFrame(frame,focusMeasure);
    bool_msg.header.stamp=now;    
    
    sensor_msgs::Image rgbRosMsg;
    sensor_msgs::Image depthRosMsg;
    
    rgb->toImageMsg(rgbRosMsg);
    depth->toImageMsg(depthRosMsg);
    
    rgbRosMsg.header.stamp=now;
    depthRosMsg.header.stamp=now;
    
    frame++;
    
    image_pub.publish(rgbRosMsg);
    depth_pub.publish(depthRosMsg);
    kf_pub.publish(bool_msg);
}











int main(int argc, char *argv[])
{
    ros::init(argc, argv, "key_frame_publisher");
    ros::NodeHandle n_p("~");
    
    std::string image_topic, depth_topic,cam_info_topic;
    
    n_p.param<std::string>("image_topic", image_topic, "camera/rgb/image_rect_color");
    n_p.param<std::string>("depth_topic", depth_topic, "camera/depth_registered/sw_registered/image_rect");
    n_p.param<std::string>("cam_info_topic", cam_info_topic, "/camera/depth_registered/camera_info");
    
    n_p.param<bool>("mm_to_meters", mm_to_meters, false);        
    n_p.param<int>("kf_rate", kf_rate, 50);        
    n_p.param<double>("blurrThreshold", blurrThreshold, 150.0);

    //publishers
    image_pub =  n_p.advertise<sensor_msgs::Image>("/kfp/rgb/image_raw",100);    
    depth_pub = n_p.advertise<sensor_msgs::Image>("/kfp/depth/image_raw",100);
    cam_info_pub = n_p.advertise<sensor_msgs::CameraInfo>("/kfp/camera_info",100);
    kf_pub = n_p.advertise<key_frame_publisher::boolStamped>("/kfp/isKeyframe",100);
    
    
    ROS_INFO("Waiting camera info: %s",cam_info_topic.c_str());
    while (ros::ok())
    {
        sensor_msgs::CameraInfoConstPtr cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_info_topic);
        if (cam_info)
        {
            camInfo = *cam_info;
            break;
        }
    }
    ROS_INFO("Camera info received");
    cam_info_pub.publish(camInfo);
    
    ros::Duration(0.5).sleep();
    
    //subscribers
    message_filters::Subscriber<sensor_msgs::Image> image_sub;    
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
        
    image_sub.subscribe(n_p, image_topic, 100);
    depth_sub.subscribe(n_p, depth_topic, 100);    
    
    message_filters::Synchronizer<MySyncPolicy> ts_sync(MySyncPolicy(10), image_sub, depth_sub);
    ts_sync.registerCallback(boost::bind(&imageDepthCb, _1, _2));
    
    
    ros::spin();
}

