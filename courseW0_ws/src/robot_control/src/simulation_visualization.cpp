// Code for implementing the simulation and simple visualization for course Wheeled Robot Motion Control, that receives the motion control
// commands from the "course_w0.py" program under the "simulation/robot_motion" topic and calcucates the updated robot position on the track.
// Then, it visualizes the robot on the track, stores that under image file "viz.jpg" and publishes it under the topic "simulation/vizualization".
// dependencies: 'robot_control/RobotMotion.h','robot_control/SensorInput.h' header files for messages
//
// Maintainer: Sotirios Stasinopoulos email: sotstas@gmail.com
// Last edited: 2020.01.17


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cstdlib>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

// #include <boost/foreach.hpp>

#include <robot_control/RobotMotion.h>
#include <robot_control/SensorInput.h>


// // Extra OpenCV visualization window if we decide to exclude the final "image_view"
// static const std::string OPENCV_WINDOW = "Image window";

class SimulationVisualization
{
    ros::NodeHandle sv_nh;

    // Create image transport handle and connect it to the incoming/outgoing topics
    image_transport::ImageTransport it;
    ros::Subscriber sub_sensor_orientation;
    ros::Subscriber sub_robot_motion;
    ros::Publisher pub_sensor_input;
    image_transport::Publisher pub;

public:
    float viz_image_scaling_factor; // the scaling factor for the size of the vizualization image

    float robot_length; // length of robot in cm
    float robot_width; // width of robot in cm
    double robot_position_x; // center of robot x on track in cm
    double robot_position_y; // center of robot y on track in cm
    double robot_orientation; // orientation of robot on track in degrees
    double robot_translational_speed; // translational speed of robot in cm/s along the orientation axis
    double robot_rotational_speed; // rotational speed of robot in degrees/s around its center
    double robot_start_line_x; // where the robot starts from on the x-axis
    double robot_finish_line_x; // where the robot should finish on the x-axis

    double sensor_orientation_left; // orientation of left sensor
    double sensor_orientation_front; // orientation of front sensor
    double sensor_orientation_right; // orientation of right sensor
    double sensor_displacement_left[2]; // x,y displacement of left sensor from the center of the robot
    double sensor_displacement_front[2]; // x,y displacement of front sensor from the center of the robot
    double sensor_displacement_right[2]; // x,y displacement of right sensor from the center of the robot
    double sensor_distance_range; // distance range of the ultrasonic sensors in cm
    double sensor_angular_range; // angular range of the ultrasonic sensors in degrees

    double obstacle_position[7][2]; // x,y position of the obstacles on the track, set 2 walls as obstacles
    double obstacle_size[7][2]; // length, width of the obstacles on the track
    double obstacle_length; // length of standard obstacle
    double obstacle_width; // width of standard obstacle

    double track_length; // length of the track
    double track_width; // width of the track
    double track_length_margin; // margin of the image, before and after track length
    double track_width_margin; // margin of the image, before and after track width

    double previous_time, current_time; // time keeping for calculation of robot movement
    double update_frequency;

    sensor_msgs::Image initial_image;
    cv_bridge::CvImagePtr cv_ptr;


    SimulationVisualization()
        : it(sv_nh)
    {
        sub_sensor_orientation = sv_nh.subscribe("simulation/sensor_orientation", 10, &SimulationVisualization::simulationInitializationCb, this);
        sub_robot_motion = sv_nh.subscribe("simulation/robot_motion", 10, &SimulationVisualization::newRobotMotionCb, this);
        pub = it.advertise("simulation/visualization", 10);
        pub_sensor_input = sv_nh.advertise<robot_control::SensorInput>("simulation/sensor_input",10);

        // cv::namedWindow(OPENCV_WINDOW);

        // Initialization of visualization image scaling factor
        viz_image_scaling_factor = 2;

        // Initializations for robot
        robot_length = 28;
        robot_width = 16;
        robot_position_x = -robot_length/2; // in cm, x positive to the front, relative to start line of the track
        robot_position_y = 50; // in cm, y positive to the right, relative to the right wall of track
        robot_orientation = 0; // in degrees, positive clockwise, relative to orientation perpendicular to the start line of the track
        robot_translational_speed = 0;
        robot_rotational_speed = 0;


        // Initialization of the sensors on the robot
        sensor_orientation_left = -15; // in degrees, positive clockwise, relative to front orientation of the robot
        sensor_orientation_front = 0;
        sensor_orientation_right = 15;
        sensor_displacement_left[0] = 11; // in cm, x positive to the front, relative to center
        sensor_displacement_left[1] = 5; // in cm, y positive to the left, relative to center
        sensor_displacement_front[0] = 15;
        sensor_displacement_front[1] = 0;
        sensor_displacement_right[0] = 11;
        sensor_displacement_right[1] = -5;

        sensor_distance_range = 80;
        sensor_angular_range = 15;

        // Initialization for the obstacles
        obstacle_position[0][0] = 50; // in cm, x positive to the front, relative to start line of track
        obstacle_position[0][1] = 50; // in cm, y positive to the right, relative to the right wall of track
        obstacle_position[1][0] = 110;
        obstacle_position[1][1] = 80;
        obstacle_position[2][0] = 130;
        obstacle_position[2][1] = 20;
        obstacle_position[3][0] = 190;
        obstacle_position[3][1] = 40;
        obstacle_position[4][0] = 250;
        obstacle_position[4][1] = 75;
        obstacle_position[5][0] = 150; // right wall
        obstacle_position[5][1] = 0;
        obstacle_position[6][0] = 150; // left wall
        obstacle_position[6][1] = 100;

        obstacle_length = 5;
        obstacle_width = 5;

        obstacle_size[0][0] = obstacle_length; // in cm, length parallel to x axis of track
        obstacle_size[0][1] = obstacle_width; // in cm, width parallel to y axis of track
        obstacle_size[1][0] = obstacle_length;
        obstacle_size[1][1] = obstacle_width;
        obstacle_size[2][0] = obstacle_length;
        obstacle_size[2][1] = obstacle_width;
        obstacle_size[3][0] = obstacle_length;
        obstacle_size[3][1] = obstacle_width;
        obstacle_size[4][0] = obstacle_length;
        obstacle_size[4][1] = obstacle_width;
        obstacle_size[5][0] = obstacle_length; // right wall spans the entire track
        obstacle_size[5][1] = obstacle_width;
        obstacle_size[6][0] = obstacle_length; // left wall spans the entire track
        obstacle_size[6][1] = obstacle_width;

        // Initialization of track
        track_length = 300; // in cm, x positive to the front, relative to start line of track
        track_width = 100; // in cm, y positive to the right, relative to the left wall of track
        track_length_margin = 40;
        track_width_margin = 20;

        robot_start_line_x = -robot_length/2;
        robot_finish_line_x = track_length+robot_length/2;

        // Initialize time
        previous_time = ros::Time::now().toSec();
        current_time = ros::Time::now().toSec();
        update_frequency = 5; // 5Hz, step time 0.2sec

        // Initialization of track image
        initial_image.header.stamp = ros::Time::now();
        initial_image.header.frame_id = "/track";
        initial_image.height = track_length + 2*track_length_margin; // image height = track_length + 20cm on each side
        initial_image.width = track_width + 2*track_width_margin; // image width = track_width + 40cm on each side
        initial_image.encoding = sensor_msgs::image_encodings::BGR8;
        //        ROS_INFO("height: [%d] width: [%d] encoding: [%s]", initial_image.height, initial_image.width, initial_image.encoding.c_str());
        initial_image.is_bigendian = true;
        initial_image.step = 3 * initial_image.width; // line in bytes = 3 colors of 1 byte per pixel
        initial_image.data.resize(initial_image.height*initial_image.step);


        // Create images compatible with OpenCV using cv_bridge
        try
        {
            cv_ptr = cv_bridge::toCvCopy(initial_image, "bgr8");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Draw image with white background
        cv_ptr->image = cv::Mat::ones(cv_ptr->image.rows,cv_ptr->image.cols,CV_8UC3);

        //     // Update GUI Window
        //     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        //     cv::waitKey(3);

        // Store image to the stream file
        cv::imwrite("/home/ubuntu/camera_stream.jpg",cv_ptr->image);

        //Publishing the drawn image
        if (sensor_msgs::ImagePtr sim_viz_image = cv_ptr->toImageMsg())
        {
            pub.publish(sim_viz_image);
        }
        else
            fprintf(stderr,"Couldn't publish visualization image");
    }

    ~SimulationVisualization()
    {
        //    cv::destroyWindow(OPENCV_WINDOW);
    }


    // Callback function for initializing the simulation with the sensor orientation
    void simulationInitializationCb(const robot_control::SensorInputConstPtr& sensor_orientation_msg)
    {

        sensor_orientation_left = sensor_orientation_msg->left_sensor_distance;
        sensor_orientation_front = sensor_orientation_msg->front_sensor_distance;
        sensor_orientation_right = sensor_orientation_msg->right_sensor_distance;

        // Drawing black track on white background
        cv::rectangle(cv_ptr->image,cv::Point(0,0),
                      cv::Point(2*track_width_margin+track_width,2*track_length_margin+track_length),
                      cv::Scalar(255,255,255),-1);
        cv::rectangle(cv_ptr->image,cv::Point(track_width_margin,track_length_margin),
                      cv::Point(track_width_margin+track_width,track_length_margin+track_length),
                      cv::Scalar(127,127,127),2);

        // Drawing the Start and Finish lines
        cv::putText(cv_ptr->image,"START",cv::Point(track_width_margin-10,track_length_margin-10),
                    cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,0,0));
        cv::putText(cv_ptr->image,"FINISH",cv::Point(track_width_margin-10,track_length_margin+track_length+20),
                    cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,0,0));


//        // Drawing obstacles with brown
//        for (int i=0;i<7;i++)
//        {
//            cv::rectangle(cv_ptr->image,cv::Point(track_width_margin+obstacle_position[i][1]-obstacle_size[i][1]/2,
//                    track_length_margin+obstacle_position[i][0]-obstacle_size[i][0]/2),
//                    cv::Point(track_width_margin+obstacle_position[i][1]+obstacle_size[i][1]/2,
//                    track_length_margin+obstacle_position[i][0]+obstacle_size[i][0]/2),
//                    cv::Scalar(42,42,165),-1);
//        }

        // Drawing robot with black
        cv::rectangle(cv_ptr->image,cv::Point(track_width_margin+robot_position_y-robot_width/2,
                                              track_length_margin+robot_position_x-robot_length/2),
                      cv::Point(track_width_margin+robot_position_y+robot_width/2,
                                track_length_margin+robot_position_x+robot_length/2),
                      cv::Scalar(0,0,0),-1);


//        // Drawing sensor ranges with light grey
//        // Left sensor
//        cv::line(cv_ptr->image,cv::Point(track_width_margin+robot_position_y-
//                                         sin(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                 cos(robot_orientation/180*M_PI)*sensor_displacement_left[1],
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_left[1]),
//                cv::Point(track_width_margin+robot_position_y-
//                          sin(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_left[1]-
//                sin(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                cos(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI)),
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_left[1]+
//                cos(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                sin(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))),
//                cv::Scalar(180,180,180));
//        cv::line(cv_ptr->image,cv::Point(track_width_margin+robot_position_y-
//                                         sin(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                 cos(robot_orientation/180*M_PI)*sensor_displacement_left[1],
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_left[1]),
//                cv::Point(track_width_margin+robot_position_y-
//                          sin(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_left[1]-
//                sin(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                cos(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI)),
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_left[1]+
//                cos(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                sin(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))),
//                cv::Scalar(180,180,180));

//        // Front sensor
//        cv::line(cv_ptr->image,cv::Point(track_width_margin+robot_position_y-
//                                         sin(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                 cos(robot_orientation/180*M_PI)*sensor_displacement_front[1],
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_front[1]),
//                cv::Point(track_width_margin+robot_position_y-
//                          sin(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_front[1]-
//                sin(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                cos(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI)),
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_front[1]+
//                cos(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                sin(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))),
//                cv::Scalar(180,180,180));
//        cv::line(cv_ptr->image,cv::Point(track_width_margin+robot_position_y-
//                                         sin(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                 cos(robot_orientation/180*M_PI)*sensor_displacement_front[1],
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_front[1]),
//                cv::Point(track_width_margin+robot_position_y-
//                          sin(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_front[1]-
//                sin(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                cos(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI)),
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_front[1]+
//                cos(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                sin(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))),
//                cv::Scalar(180,180,180));


//        // Right sensor
//        cv::line(cv_ptr->image,cv::Point(track_width_margin+robot_position_y-
//                                         sin(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                 cos(robot_orientation/180*M_PI)*sensor_displacement_right[1],
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_right[1]),
//                cv::Point(track_width_margin+robot_position_y-
//                          sin(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_right[1]-
//                sin(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                cos(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI)),
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_right[1]+
//                cos(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                sin(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))),
//                cv::Scalar(180,180,180));
//        cv::line(cv_ptr->image,cv::Point(track_width_margin+robot_position_y-
//                                         sin(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                 cos(robot_orientation/180*M_PI)*sensor_displacement_right[1],
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_right[1]),
//                cv::Point(track_width_margin+robot_position_y-
//                          sin(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_right[1]-
//                sin(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                cos(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI)),
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_right[1]+
//                cos(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                sin(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))),
//                cv::Scalar(180,180,180));

        previous_time = ros::Time::now().toSec();
        current_time = ros::Time::now().toSec();

        //     // Update GUI Window
        //     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        //     cv::waitKey(3);

        //        // Flip image vertically
        //        cv::flip(cv_ptr->image,cv_ptr->image,1);

        // Resizing image for vizualization
        cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(),viz_image_scaling_factor,viz_image_scaling_factor,cv::INTER_CUBIC);

        // Store image to the stream file
        cv::imwrite("/home/ubuntu/camera_stream.jpg",cv_ptr->image);

        //Publishing the drawn image
        if (sensor_msgs::ImagePtr sim_viz_image = cv_ptr->toImageMsg())
        {
            pub.publish(sim_viz_image);
        }
        else
            fprintf(stderr,"Couldn't publish visualization image");

        // Resizing image for vizualization
        cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(),1/viz_image_scaling_factor,1/viz_image_scaling_factor,cv::INTER_CUBIC);


        sleep(3);
    }

    void gameOver()
    {
        // Drawing the GAME OVER message
        cv::putText(cv_ptr->image,"GAME OVER",cv::Point(track_width_margin-10,track_length_margin+track_length/2),
                    cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,0,255));
        ROS_INFO("Game Over");

        //     // Update GUI Window
        //     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        //     cv::waitKey(3);

        // Resizing image for vizualization
        cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(),viz_image_scaling_factor,viz_image_scaling_factor,cv::INTER_CUBIC);

        // Store image to the stream file
        cv::imwrite("/home/ubuntu/camera_stream.jpg",cv_ptr->image);

        //Publishing the drawn image
        if (sensor_msgs::ImagePtr sim_viz_image = cv_ptr->toImageMsg())
        {
            pub.publish(sim_viz_image);
        }
        else
            fprintf(stderr,"Couldn't publish visualization image");

        exit(0);
    }

    void challengeComplete()
    {
        // Drawing the Congratulations message
        cv::putText(cv_ptr->image,"WELL",cv::Point(track_width_margin+25,track_length_margin+track_length/2),
                    cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,0,255));
        cv::putText(cv_ptr->image,"DONE",cv::Point(track_width_margin+25,track_length_margin+track_length/2+30),
                    cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,0,255));
        ROS_INFO("Challenge Complete");

        //     // Update GUI Window
        //     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        //     cv::waitKey(3);

        // Resizing image for vizualization
        cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(),viz_image_scaling_factor,viz_image_scaling_factor,cv::INTER_CUBIC);

        // Store image to the stream file
        cv::imwrite("/home/ubuntu/camera_stream.jpg",cv_ptr->image);

        //Publishing the drawn image
        if (sensor_msgs::ImagePtr sim_viz_image = cv_ptr->toImageMsg())
        {
            pub.publish(sim_viz_image);
        }
        else
            fprintf(stderr,"Couldn't publish visualization image");

        exit(0);
    }

    bool isRobotAtInvalidPosition()
    {

        bool isCollision = false;

        cv::RotatedRect rRect_robot = cv::RotatedRect(cv::Point2f(robot_position_x,robot_position_y),
                                                      cv::Point2f(robot_length,robot_width),robot_orientation);

        // Check if robot is out of the boundaries of the track
        if (robot_position_x<robot_start_line_x || robot_position_y<0 || robot_position_y>track_width)
        {
            ROS_INFO("Robot out of track");
            isCollision = true;
        }

        // Check for each obstacle
        for (int i=0;i<7;i++)
        {
            cv::RotatedRect rRect_obstacle = cv::RotatedRect(cv::Point2f(obstacle_position[i][0],obstacle_position[i][1]),
                    cv::Point2f(obstacle_size[i][0],obstacle_size[i][1]),0);

            // Find the intersection of the convex areas
            cv::Mat IntersectionAreaMat(cv::Mat::zeros(8,2,CV_32FC2));
            cv::OutputArray IntersectionArea(IntersectionAreaMat);
            if (cv::rotatedRectangleIntersection(rRect_robot,rRect_obstacle,IntersectionArea)!=cv::INTERSECT_NONE)
            {
                isCollision = true;
                IntersectionAreaMat = IntersectionArea.getMat();
                for( int j = 0; j < IntersectionAreaMat.rows; j++ )
                {

                    ROS_INFO("Collision detected with obstacle %d at intersection point %d:(%f,%f)",
                             i,j,IntersectionAreaMat.at<float>(j,0),IntersectionAreaMat.at<float>(j,1));

                }
            }
        }


        return isCollision;

    }

    // Callback function that creates the track and objects and draws upon it the robot motion
    void newRobotMotionCb(const robot_control::RobotMotionConstPtr& robot_motion_msg)
    {

        robot_control::RobotMotion local_robot_motion;
        local_robot_motion.motion_command = robot_motion_msg->motion_command;
        local_robot_motion.robot_speed = robot_motion_msg->robot_speed;


        // Calculate robot translational and rotational speed according to command and time interval
        switch (local_robot_motion.motion_command) {
        case 0:
            robot_translational_speed=0;
            robot_rotational_speed=0;
            break;
        case 1:
            robot_translational_speed=local_robot_motion.robot_speed;
            robot_rotational_speed=0;
            break;
        case 2:
            robot_translational_speed=-local_robot_motion.robot_speed;
            robot_rotational_speed=0;
            break;
        case 3:
            robot_translational_speed=0;
            robot_rotational_speed=-local_robot_motion.robot_speed;
            break;
        case 4:
            robot_translational_speed=0;
            robot_rotational_speed=local_robot_motion.robot_speed;
            break;
        default:
            robot_translational_speed=0;
            robot_rotational_speed=0;
        }

    }


    robot_control::SensorInput obstacleDistanceFromSensors()
    {
        //ROS_INFO("Inside the obstacleDistanceFromSensors function");
        robot_control::SensorInput local_sensor_input_msg;
        local_sensor_input_msg.left_sensor_distance = 500;
        local_sensor_input_msg.front_sensor_distance = 500;
        local_sensor_input_msg.right_sensor_distance = 500;

        for (int i=0;i<3;i++)
        {
            double sensor_displacement[2];
            double sensor_orientation;
            switch (i) {
            case 0:
                sensor_displacement[0] = sensor_displacement_left[0];
                sensor_displacement[1] = sensor_displacement_left[1];
                sensor_orientation = sensor_orientation_left;
                break;
            case 1:
                sensor_displacement[0] = sensor_displacement_front[0];
                sensor_displacement[1] = sensor_displacement_front[1];
                sensor_orientation = sensor_orientation_front;
                break;
            case 2:
                sensor_displacement[0] = sensor_displacement_right[0];
                sensor_displacement[1] = sensor_displacement_right[1];
                sensor_orientation = sensor_orientation_right;
                break;
            }

            //            // Store the 3 points of the sensor range in an array
            //            double sensor_range_area[3][2];
            //            sensor_range_area[0][0] = track_width_margin+robot_position_y-
            //                    sin(robot_orientation/180*M_PI)*sensor_displacement[0]+
            //                    cos(robot_orientation/180*M_PI)*sensor_displacement[1];
            //            sensor_range_area[0][1] = track_length_margin+robot_position_x+
            //                    cos(robot_orientation/180*M_PI)*sensor_displacement[0]+
            //                    sin(robot_orientation/180*M_PI)*sensor_displacement[1];
            //            sensor_range_area[1][0] = track_width_margin+robot_position_y-
            //                    sin(robot_orientation/180*M_PI)*sensor_displacement[0]+
            //                    cos(robot_orientation/180*M_PI)*sensor_displacement[1]-
            //                    sin(robot_orientation/180*M_PI)*
            //                    (cos(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
            //                     sin(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
            //                    cos(robot_orientation/180*M_PI)*
            //                    (sin(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
            //                     cos(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI));
            //            sensor_range_area[1][1] = track_length_margin+robot_position_x+
            //                    cos(robot_orientation/180*M_PI)*sensor_displacement[0]+
            //                    sin(robot_orientation/180*M_PI)*sensor_displacement[1]+
            //                    cos(robot_orientation/180*M_PI)*
            //                    (cos(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
            //                     sin(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
            //                    sin(robot_orientation/180*M_PI)*
            //                    (sin(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
            //                     cos(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI));
            //            sensor_range_area[2][0] = track_width_margin+robot_position_y-
            //                    sin(robot_orientation/180*M_PI)*sensor_displacement[0]+
            //                    cos(robot_orientation/180*M_PI)*sensor_displacement[1]-
            //                    sin(robot_orientation/180*M_PI)*
            //                    (cos(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
            //                     sin(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
            //                    cos(robot_orientation/180*M_PI)*
            //                    (sin(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
            //                     cos(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI));
            //            sensor_range_area[2][1] = track_length_margin+robot_position_x+
            //                    cos(robot_orientation/180*M_PI)*sensor_displacement[0]+
            //                    sin(robot_orientation/180*M_PI)*sensor_displacement[1]+
            //                    cos(robot_orientation/180*M_PI)*
            //                    (cos(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
            //                     sin(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
            //                    sin(robot_orientation/180*M_PI)*
            //                    (sin(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
            //                     cos(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI));

            //            // Convert to Mat
            //            cv::Mat SensorMat(3,2,CV_64F);
            //            std::memcpy(SensorMat.data, sensor_range_area, 3*2*sizeof(double));


            // Store the 3 points of the sensor range in a Point vector
            std::vector<cv::Point> sensor_range_area;
            sensor_range_area.push_back(cv::Point(
                                            robot_position_x+
                                            cos(robot_orientation/180*M_PI)*sensor_displacement[0]+
                                        sin(robot_orientation/180*M_PI)*sensor_displacement[1],
                    robot_position_y-
                    sin(robot_orientation/180*M_PI)*sensor_displacement[0]+
                    cos(robot_orientation/180*M_PI)*sensor_displacement[1]));
            sensor_range_area.push_back(cv::Point(
                                            robot_position_x+
                                            cos(robot_orientation/180*M_PI)*sensor_displacement[0]+
                                        sin(robot_orientation/180*M_PI)*sensor_displacement[1]+
                    cos(robot_orientation/180*M_PI)*
                    (cos(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
                     sin(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
                    sin(robot_orientation/180*M_PI)*
                    (sin(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
                     cos(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI)),
                    robot_position_y-
                    sin(robot_orientation/180*M_PI)*sensor_displacement[0]+
                    cos(robot_orientation/180*M_PI)*sensor_displacement[1]-
                    sin(robot_orientation/180*M_PI)*
                    (cos(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
                     sin(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
                    cos(robot_orientation/180*M_PI)*
                    (sin(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
                     cos(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))));
            sensor_range_area.push_back(cv::Point(
                                            robot_position_x+
                                            cos(robot_orientation/180*M_PI)*sensor_displacement[0]+
                                        sin(robot_orientation/180*M_PI)*sensor_displacement[1]+
                    cos(robot_orientation/180*M_PI)*
                    (cos(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
                     sin(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
                    sin(robot_orientation/180*M_PI)*
                    (sin(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
                     cos(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI)),
                    robot_position_y-
                    sin(robot_orientation/180*M_PI)*sensor_displacement[0]+
                    cos(robot_orientation/180*M_PI)*sensor_displacement[1]-
                    sin(robot_orientation/180*M_PI)*
                    (cos(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
                     sin(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
                    cos(robot_orientation/180*M_PI)*
                    (sin(sensor_orientation/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
                     cos(sensor_orientation/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))));

//            ROS_INFO("Sensor %d range points (%d,%d),(%d,%d),(%d,%d)",i,sensor_range_area.at(0).x,sensor_range_area.at(0).y,
//                     sensor_range_area.at(1).x,sensor_range_area.at(1).y,sensor_range_area.at(2).x,sensor_range_area.at(2).y);


            double min_dist = 80;
            bool intersection_found = false;
            // Check for each obstacle
            for (int j=0;j<7;j++)
            {
                //                // Store the 4 points of obstacle bounding box in an array
                //                double obstacle_area[4][2];
                //                obstacle_area[0][0] = obstacle_position[j][0]-obstacle_size[j][0]/2;
                //                obstacle_area[0][1] = obstacle_position[j][1]-obstacle_size[j][1]/2;
                //                obstacle_area[1][0] = obstacle_position[j][0]-obstacle_size[j][0]/2;
                //                obstacle_area[1][1] = obstacle_position[j][1]+obstacle_size[j][1]/2;
                //                obstacle_area[2][0] = obstacle_position[j][0]+obstacle_size[j][0]/2;
                //                obstacle_area[2][1] = obstacle_position[j][1]-obstacle_size[j][1]/2;
                //                obstacle_area[3][0] = obstacle_position[j][0]+obstacle_size[j][0]/2;
                //                obstacle_area[3][1] = obstacle_position[j][1]+obstacle_size[j][1]/2;

                //                // Convert to Mat
                //                cv::Mat ObstacleMat(4,2,CV_64F);
                //                std::memcpy(ObstacleMat.data, obstacle_area, 4*2*sizeof(double));

                // Store the 4 points of obstacle bounding box in a Point vector
                std::vector<cv::Point> obstacle_area;
                obstacle_area.push_back(cv::Point(obstacle_position[j][0]-obstacle_size[j][0]/2,
                        obstacle_position[j][1]-obstacle_size[j][1]/2));
                obstacle_area.push_back(cv::Point(obstacle_position[j][0]-obstacle_size[j][0]/2,
                        obstacle_position[j][1]+obstacle_size[j][1]/2));
                obstacle_area.push_back(cv::Point(obstacle_position[j][0]+obstacle_size[j][0]/2,
                        obstacle_position[j][1]-obstacle_size[j][1]/2));
                obstacle_area.push_back(cv::Point(obstacle_position[j][0]+obstacle_size[j][0]/2,
                        obstacle_position[j][1]+obstacle_size[j][1]/2));

                //                ROS_INFO("Obstacle points (%d,%d),(%d,%d),(%d,%d),(%d,%d)",obstacle_area.at(0).x,obstacle_area.at(0).y,
                //                         obstacle_area.at(1).x,obstacle_area.at(1).y,obstacle_area.at(2).x,obstacle_area.at(2).y,obstacle_area.at(3).x,obstacle_area.at(3).y);

                // Find the intersection of the convex areas
                //                cv::Mat IntersectionAreaMat(cv::Mat::zeros(8,2,CV_32FC2));
                //                cv::OutputArray IntersectionArea(IntersectionAreaMat);

                //                cv::intersectConvexConvex(cv::InputArray(SensorMat),cv::InputArray(ObstacleMat),IntersectionArea);

                //                IntersectionAreaMat = IntersectionArea.getMat();

                //                for( int k = 0; k < IntersectionAreaMat.rows; k++ )
                //                {

                //                    if (IntersectionAreaMat.at<float>(k,0)!=0)
                //                    {
                //                        double sensor_obstacle_contour_point_dist;
                //                        sensor_obstacle_contour_point_dist = cv::norm(cv::Point(sensor_range_area[0][0],sensor_range_area[0][1])-
                //                                cv::Point(IntersectionAreaMat.at<float>(k,0),IntersectionAreaMat.at<float>(k,1)));
                //                        if (sensor_obstacle_contour_point_dist < min_dist)
                //                            min_dist = sensor_obstacle_contour_point_dist;
                //                    }

                //                }

                // Find the intersection polygon of the sensor range triangle and the obstacles
                std::vector<cv::Point> intersectionPolygon;
                //float intersectArea =cv::intersectConvexConvex(cv::InputArray(SensorMat),cv::InputArray(ObstacleMat),intersectionPolygon);
                float intersectArea =cv::intersectConvexConvex(sensor_range_area,obstacle_area,intersectionPolygon);

                // If the intersection area exists then check distance of intersection from sensor
                if (intersectArea>0)
                {
                    intersection_found = true;
                    for( int k = 0; k < intersectionPolygon.size(); k++ )
                    {

                        //                        if (intersectionPolygon.at(k).x!=0)
                        //                        {
                        double sensor_obstacle_contour_point_dist;
                        sensor_obstacle_contour_point_dist = cv::norm(cv::Point(sensor_range_area.at(0).x,sensor_range_area.at(0).y)-
                                                                      cv::Point(intersectionPolygon.at(k).x,intersectionPolygon.at(k).y));

//                        ROS_INFO("Obstacle %d in range of sensor %d, intersection polygon point %d (%d,%d), distance from sensor %f",
//                                 j,i,k,intersectionPolygon.at(k).x,intersectionPolygon.at(k).y,sensor_obstacle_contour_point_dist);

                        // If the distance is 0, it means that an intersection point was calculated wrongly
                        if (sensor_obstacle_contour_point_dist < min_dist && sensor_obstacle_contour_point_dist!=0.0)
                            min_dist = sensor_obstacle_contour_point_dist;
                        //                        }

                    }
                    ROS_INFO("Obstacle %d in range of sensor %d",j,i);
                }


            }
            if (intersection_found && (min_dist <80))
            {
                switch (i) {
                case 0:
                    local_sensor_input_msg.left_sensor_distance = min_dist;
                    ROS_INFO("Distance from left sensor is %f",min_dist);
                    break;
                case 1:
                    local_sensor_input_msg.front_sensor_distance = min_dist;
                    ROS_INFO("Distance from front sensor is %f",min_dist);
                    break;
                case 2:
                    local_sensor_input_msg.right_sensor_distance = min_dist;
                    ROS_INFO("Distance from right sensor is %f",min_dist);
                    break;
                }
            }
            else
            {
                switch (i) {
                case 0:
                    local_sensor_input_msg.left_sensor_distance = 500;
                    //ROS_INFO("No obstacle in range of left sensor");
                    break;
                case 1:
                    local_sensor_input_msg.front_sensor_distance = 500;
                    //ROS_INFO("No obstacle in range of front sensor");
                    break;
                case 2:
                    local_sensor_input_msg.right_sensor_distance = 500;
                    //ROS_INFO("No obstacle in range of right sensor");
                    break;
                }
            }
        }
        return local_sensor_input_msg;
    }

    void updateRobotPositionAndSensorInput()
    {


        // Update simulation vizualization image
        // Drawing black track on white background
        cv::rectangle(cv_ptr->image,cv::Point(0,0),
                      cv::Point(2*track_width_margin+track_width,2*track_length_margin+track_length),
                      cv::Scalar(255,255,255),-1);
        cv::rectangle(cv_ptr->image,cv::Point(track_width_margin,track_length_margin),
                      cv::Point(track_width_margin+track_width,track_length_margin+track_length),
                      cv::Scalar(127,127,127),2);

        // Drawing the Start and Finish lines
        cv::putText(cv_ptr->image,"START",cv::Point(track_width_margin-10,track_length_margin-10),
                    cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,0,0));
        cv::putText(cv_ptr->image,"FINISH",cv::Point(track_width_margin-10,track_length_margin+track_length+20),
                    cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,0,0));

//        // Drawing obstacles with brown
//        for (int i=0;i<7;i++)
//        {
//            cv::rectangle(cv_ptr->image,cv::Point(track_width_margin+obstacle_position[i][1]-obstacle_size[i][1]/2,
//                    track_length_margin+obstacle_position[i][0]-obstacle_size[i][0]/2),
//                    cv::Point(track_width_margin+obstacle_position[i][1]+obstacle_size[i][1]/2,
//                    track_length_margin+obstacle_position[i][0]+obstacle_size[i][0]/2),
//                    cv::Scalar(42,42,165),-1);
//        }

        // Calculate new robot position and orientation
        robot_position_x = robot_position_x + robot_translational_speed * cos(robot_orientation/180*M_PI) * 1/update_frequency;\
        robot_position_y = robot_position_y - robot_translational_speed * sin(robot_orientation/180*M_PI) * 1/update_frequency;
        robot_orientation = robot_orientation + robot_rotational_speed * 1/update_frequency;

        //        ROS_INFO("robot_position_x:%f, robot_position_y:%f, robot_orientation:%f, robot_translational_speed:%f, robot_rotational_speed:%f",
        //                 robot_position_x,robot_position_y,robot_orientation,robot_translational_speed,robot_rotational_speed);

        // Drawing robot with black
        cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(track_width_margin+robot_position_y,track_length_margin+robot_position_x),
                                                cv::Point2f(robot_width,robot_length),robot_orientation);
        cv::Point2f vertices[4];
        rRect.points(vertices);
        for (int i = 0; i < 4; i++)
            cv::line(cv_ptr->image, vertices[i], vertices[(i+1)%4], cv::Scalar(0,0,0),2);

        cv::Rect brect = rRect.boundingRect();
        //cv::rectangle(cv_ptr->image,brect,cv::Scalar(0,0,0));



//        // Drawing sensor ranges with light grey
//        // Left sensor
//        cv::line(cv_ptr->image,cv::Point(track_width_margin+robot_position_y-
//                                         sin(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                 cos(robot_orientation/180*M_PI)*sensor_displacement_left[1],
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_left[1]),
//                cv::Point(track_width_margin+robot_position_y-
//                          sin(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_left[1]-
//                sin(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                cos(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI)),
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_left[1]+
//                cos(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                sin(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))),
//                cv::Scalar(180,180,180));
//        cv::line(cv_ptr->image,cv::Point(track_width_margin+robot_position_y-
//                                         sin(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                 cos(robot_orientation/180*M_PI)*sensor_displacement_left[1],
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_left[1]),
//                cv::Point(track_width_margin+robot_position_y-
//                          sin(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_left[1]-
//                sin(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                cos(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI)),
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_left[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_left[1]+
//                cos(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                sin(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_left/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 cos(sensor_orientation_left/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))),
//                cv::Scalar(180,180,180));

//        // Front sensor
//        cv::line(cv_ptr->image,cv::Point(track_width_margin+robot_position_y-
//                                         sin(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                 cos(robot_orientation/180*M_PI)*sensor_displacement_front[1],
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_front[1]),
//                cv::Point(track_width_margin+robot_position_y-
//                          sin(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_front[1]-
//                sin(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                cos(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI)),
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_front[1]+
//                cos(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                sin(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))),
//                cv::Scalar(180,180,180));
//        cv::line(cv_ptr->image,cv::Point(track_width_margin+robot_position_y-
//                                         sin(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                 cos(robot_orientation/180*M_PI)*sensor_displacement_front[1],
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_front[1]),
//                cv::Point(track_width_margin+robot_position_y-
//                          sin(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_front[1]-
//                sin(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                cos(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI)),
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_front[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_front[1]+
//                cos(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                sin(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_front/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 cos(sensor_orientation_front/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))),
//                cv::Scalar(180,180,180));


//        // Right sensor
//        cv::line(cv_ptr->image,cv::Point(track_width_margin+robot_position_y-
//                                         sin(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                 cos(robot_orientation/180*M_PI)*sensor_displacement_right[1],
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_right[1]),
//                cv::Point(track_width_margin+robot_position_y-
//                          sin(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_right[1]-
//                sin(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                cos(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI)),
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_right[1]+
//                cos(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                sin(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))),
//                cv::Scalar(180,180,180));
//        cv::line(cv_ptr->image,cv::Point(track_width_margin+robot_position_y-
//                                         sin(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                 cos(robot_orientation/180*M_PI)*sensor_displacement_right[1],
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_right[1]),
//                cv::Point(track_width_margin+robot_position_y-
//                          sin(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_right[1]-
//                sin(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                cos(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI)),
//                track_length_margin+robot_position_x+
//                cos(robot_orientation/180*M_PI)*sensor_displacement_right[0]+
//                sin(robot_orientation/180*M_PI)*sensor_displacement_right[1]+
//                cos(robot_orientation/180*M_PI)*
//                (cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)+
//                 sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))-
//                sin(robot_orientation/180*M_PI)*
//                (sin(sensor_orientation_right/180*M_PI)*sensor_distance_range*cos(sensor_angular_range/2/180*M_PI)-
//                 cos(sensor_orientation_right/180*M_PI)*sensor_distance_range*sin(sensor_angular_range/2/180*M_PI))),
//                cv::Scalar(180,180,180));


//        // Check if the new robot position is invalid
//        if (isRobotAtInvalidPosition())
//        {
//            gameOver();
//        }

        // Check if robot has passed the finish line
        if (robot_position_x>robot_finish_line_x) // if the robot has passed the finish line completely
        {
            challengeComplete();
        }

        //     // Update GUI Window
        //     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        //     cv::waitKey(3);

        // Resizing image for vizualization
        cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(),viz_image_scaling_factor,viz_image_scaling_factor,cv::INTER_CUBIC);


        // Store image to the stream file
        cv::imwrite("/home/ubuntu/camera_stream.jpg",cv_ptr->image);


        //Publishing the drawn image
        if (sensor_msgs::ImagePtr sim_viz_image = cv_ptr->toImageMsg())
        {
            pub.publish(sim_viz_image);
        }
        else
            fprintf(stderr,"Couldn't publish visualization image");

        // Resizing image for vizualization
        cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(),1/viz_image_scaling_factor,1/viz_image_scaling_factor,cv::INTER_CUBIC);

//        // Update sensor input
//        robot_control::SensorInput sensor_input_msg;
//        sensor_input_msg = obstacleDistanceFromSensors();
//        pub_sensor_input.publish(sensor_input_msg);

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulation_visualization");

    SimulationVisualization sv;

    //ROS_INFO("Before calling sleep");
    ros::Rate rate(sv.update_frequency); // because step period is 1/20 = 0.05sec
    rate.sleep(); // delay before initializing

    while(ros::ok())
    {
        sv.updateRobotPositionAndSensorInput();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
