#include <object_detector_cdt/object_detector.h>
#include <iostream>
#include <cmath>
// parameters
int count = 0;
std::string folder_path = "/home/yifu/tmp/cdt_drs/";

ObjectDetector::ObjectDetector(ros::NodeHandle &nh)
{
    // Read parameters
    readParameters(nh);

    // Setup subscriber
    image_transport::ImageTransport it(nh);
    image_sub_ = it.subscribe(input_image_topic_, 1, &ObjectDetector::imageCallback, this);

    // Setup publisher
    objects_pub_ = nh.advertise<cdt_msgs::ObjectList>(output_objects_topic_, 10);

    // Extrinsic calibration. This must be updated accordingly
    camera_extrinsic_x_ = 0.2;
    camera_extrinsic_y_ = 0.2;
    camera_extrinsic_z_ = 0.0;

    // Intrinsic calibration
    camera_fx_ = 381.3;
    camera_fy_ = 381.3;
    camera_cx_ = 320.5;
    camera_cy_ = 240.5;

    // Real heights of objects
    barrel_real_height_     = 1.2;   // meters 
    barrow_real_height_     = 0.7;   // meters, note: includes the wheel and frame 
    computer_real_height_   = 0.5;   // meters 
    dog_real_height_        = 0.418; // meters, note: includes legs 
}

void ObjectDetector::readParameters(ros::NodeHandle &nh)
{
    // Depending on the parameter (required or optional) the API differs:

    // input_topic is required (no default topic)
    if (!nh.getParam("input_image_topic", input_image_topic_))
    {
        ROS_ERROR("Could not read parameter `input_topic`.");
        exit(-1);
    }

    if (!nh.getParam("input_base_frame", base_frame_))
    {
        ROS_ERROR("Could not read parameter `input_base_frame`.");
        exit(-1);
    }
    if (!nh.getParam("input_fixed_frame", fixed_frame_))
    {
        ROS_ERROR("Could not read parameter `goal_frame`.");
        exit(-1);
    }   

    // output topic is optional. It will use '/detected_objects' by default
    nh.param("output_objects_topic", output_objects_topic_, std::string("/detected_objects"));
}

void ObjectDetector::imageCallback(const sensor_msgs::ImageConstPtr &in_msg)
{
    ROS_DEBUG("New image received!");

    // Preallocate some variables
    cv::Mat image;
    ros::Time timestamp;

    double x, y, theta;
    getRobotPose(x, y, theta);

    // Convert message to OpenCV image
    convertMessageToImage(in_msg, image, timestamp);

    // Recognize object
    // Dog
    // TODO: This only publishes the first time we detect the dog
    // if(!wasObjectDetected("dog")) // TODO: implement a better check
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeObject(image, timestamp, x, y, theta, new_object,"dog");

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            detected_objects_.objects.push_back(new_object);
        }
    }
    // TODO: Add other objects here


    // Publish list of objects detected so far
    objects_pub_.publish(detected_objects_);
}

void ObjectDetector::convertMessageToImage(const sensor_msgs::ImageConstPtr &in_msg, cv::Mat &out_image, ros::Time &out_timestamp)
{
    // Convert Image message to cv::Mat using cv_bridge
    out_image = cv_bridge::toCvShare(in_msg, "bgr8")->image;

    // Extract timestamp from header
    out_timestamp = in_msg->header.stamp;
}


cv::Mat ObjectDetector::applyColourFilter(const cv::Mat &in_image_bgr, const Colour &colour)
{   
    
    std::string count_str = std::to_string(count);
    std::string input_path=folder_path+"input.png";
    std::string mask_path=folder_path+"mask.png";
    std::string hsv_path=folder_path+"hsv.png";
    if (count==0){std::cout << "image saved to " + folder_path << std::endl;}
    

    // Here you should apply some binary threhsolds on the image to detect the colors
    // The output should be a binary mask indicating where the object of a given color is located
    cv::Mat mask,in_image_hsv;
    cv::cvtColor(in_image_bgr,in_image_hsv, cv::COLOR_BGR2HSV);
    if (colour == Colour::RED) {
        
        inRange(in_image_hsv, cv::Scalar(  0,  100,  0), cv::Scalar( 5, 255, 255), mask);
        // inRange(in_image_hsv, cv::Scalar(  170,  0,  0), cv::Scalar( 180, 255, 255), mask);
        // cv::Mat mask = mask1 | mask2;
    } else if (colour == Colour::YELLOW) {
        inRange(in_image_hsv, cv::Scalar(  28,  150,  0), cv::Scalar( 33, 255, 255), mask);
    } else if (colour == Colour::GREEN) {
        inRange(in_image_hsv, cv::Scalar(  40,  0,  0), cv::Scalar( 70, 255, 255), mask);
    } else if (colour == Colour::BLUE) {
        inRange(in_image_hsv, cv::Scalar(  118,  0,  0), cv::Scalar( 123, 255, 255), mask);
    } else {
        // Report color not implemented
        ROS_ERROR_STREAM("[ObjectDetector::colourFilter] colour (" << colour << "  not implemented!");
    }
    
    if (count == 2) {
        cv::imwrite(input_path, in_image_bgr);
        cv::imwrite(mask_path, mask);
        cv::imwrite(hsv_path, in_image_hsv);
    } 
    count = ((count+1)%40)+1;
    // We return the mask, that will be used later

    return mask;
}

int count_2 = 0;
cv::Mat ObjectDetector::applyBoundingBox(const cv::Mat1b &in_mask, double &x, double &y, double &width, double &height) {

    cv::Mat drawing(in_mask); // it could be useful to fill this image if you want to debug

    // TODO: Compute the bounding box using the mask
    // You need to return the center of the object in image coordinates, as well as a bounding box indicating its height and width (in pixels)
    cv::Rect bound_rect = cv::boundingRect(in_mask);
    width = double(bound_rect.width);
    height = double(bound_rect.height);
    x = bound_rect.x + width / 2.;
    y = bound_rect.y + height / 2.;

    // code for visualization
    cv::rectangle(drawing, bound_rect.tl(), bound_rect.br(), cv::Scalar(255, 0, 0), 2);
    if (count_2 == 0)
        cv::imwrite(folder_path+"bounding box.png", drawing);
    count_2 = (count_2 + 1) % 20;

    return drawing;
}

int ObjectDetector::checkBoxPosition(const double x, const double y, const double width, const double height) {
    // condition 1: a part of the object is missed from the camera
    // i.e., the bounding box is close to the image bound
    double d_x_to_center = abs(x - camera_cx_);
    double d_y_to_center = abs(y - camera_cy_);

    if (((x - width / 2. < 5.) or (x + width / 2. > 640. - 5.)) || ((y - height / 2. < 5.) or (y + height / 2. > 480. - 5.)))
    {
        // std::cout << "Only a part of the object is in the image" << std::endl;
        return 1;
    }

    // condition 2: the object should not be far away from the camera
    // i.e., width and height should be large enough
    else if ((width < 640 / 20.) || (height < 480. / 20.))
    {
        // std::cout << "The object is not close enough to the camera" << std::endl;
        return 2;
    }

    // condition 3: the object should be close to the center of the image
    else if ((d_x_to_center > 640. / 20.) || (d_y_to_center > 480. / 20.))
    {
        // std::cout << "The object is not close enough to the image center" << std::endl;
        return 3;
    }

    else {
        // std::cout << "Successfully detect" << std::endl;
        return 0;
    }

}

bool ObjectDetector::recognizeObject(const cv::Mat &in_image, const ros::Time &in_timestamp, 
                                  const double& robot_x, const double& robot_y, const double& robot_theta,
                                  cdt_msgs::Object &out_new_object, const std::string object_class)
{
    // The values below will be filled by the following functions
    double dog_image_center_x;
    double dog_image_center_y;
    double dog_image_height;
    double dog_image_width;

    // TODO: the functions we use below should be filled to make this work
    cv::Mat in_image_red = applyColourFilter(in_image, Colour::RED);
    // cv::Mat in_image_red = applyColourFilter(in_image, Colour::YELLOW);
    cv::Mat in_image_bounding_box = applyBoundingBox(in_image_red, dog_image_center_x, dog_image_center_y, dog_image_width, dog_image_height);
    int check_box = checkBoxPosition(dog_image_center_x, dog_image_center_y, dog_image_width, dog_image_height);
    // std::cout << object_class << std::endl;
    if (check_box!=0)
        return false;
    // Note: Almost everything below should be kept as it is

    // We convert the image position in pixels into "real" coordinates in the camera frame
    // We use the intrinsics to compute the depth
    double depth = dog_real_height_ / dog_image_height * camera_fy_;
    //{std::cout << "depth " << depth << object_class << " " << height << dog_image_height << std::endl;}
    // We now back-project the center using the  pinhole camera model
    // The result is in camera coordinates. Camera coordinates are weird, see note below
    double dog_position_camera_x = depth / camera_fx_ * (dog_image_center_x - camera_cx_);
    double dog_position_camera_y = depth / camera_fy_ * (dog_image_center_y - camera_cy_);
    double dog_position_camera_z = depth;


    // Camera coordinates are different to robot and fixed frame coordinates
    // Robot and fixed frame are x forward, y left and z upward
    // Camera coordinates are x right, y downward, z forward
    // robot x -> camera  z 
    // robot y -> camera -x
    // robot z -> camera -y
    // They follow x-red, y-green and z-blue in both cases though
    
    double dog_position_base_x = (camera_extrinsic_x_ +  dog_position_camera_z);
    double dog_position_base_y = (camera_extrinsic_y_ + -dog_position_camera_x);
    
    // We need to be careful when computing the final position of the object in global (fixed frame) coordinates
    // We need to introduce a correction givne by the robot orientation
    // Fill message
    out_new_object.id = object_class;
    out_new_object.header.stamp = in_timestamp;
    out_new_object.header.frame_id = fixed_frame_;
    out_new_object.position.x = robot_x +  cos(robot_theta)*dog_position_base_x + sin(-robot_theta) * dog_position_base_y;
    out_new_object.position.y = robot_y +  sin(robot_theta)*dog_position_base_x + cos(robot_theta) * dog_position_base_y;
    out_new_object.position.z = 0.0     + camera_extrinsic_z_ + -dog_position_camera_y;

    return std::isfinite(depth);
}

// TODO: Implement similar methods for other objects
// HERE


// Utils
void ObjectDetector::getRobotPose(double &x, double &y, double &theta)
{
    // Get current pose
    tf::StampedTransform base_to_map_transform;
    tf_listener_.waitForTransform(fixed_frame_, base_frame_,  ros::Time(0), ros::Duration(0.5));
    try
    {
        tf_listener_.lookupTransform(fixed_frame_, base_frame_, ros::Time(0), base_to_map_transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    // Extract components from robot pose
    x = base_to_map_transform.getOrigin().getX();
    y = base_to_map_transform.getOrigin().getY();

    // Extract orientation is more involved, since it is a quaternion
    // We'll get some help from Eigen
    // First we create an Eigen quaternion
    Eigen::Quaterniond q(base_to_map_transform.getRotation().getW(),
                         base_to_map_transform.getRotation().getX(),
                         base_to_map_transform.getRotation().getY(),
                         base_to_map_transform.getRotation().getZ());
    // We convert it to an Axis-Angle representation
    // This representation is given by an axis wrt to some coordinate frame, and a rotation along that axis
    Eigen::AngleAxisd axis_angle(q);

    // The value corresponding to the z component is the orientation wrt to the z axis (planar rotation)
    // We need to extract the z component of the axis and multiply it by the angle
    theta = axis_angle.axis().z() * axis_angle.angle();
}

bool ObjectDetector::wasObjectDetected(std::string object_name)
{
    bool detected = false;
    for(auto obj : detected_objects_.objects)
    {
        if(obj.id == object_name)
            detected = true;
    }

    return detected;
}