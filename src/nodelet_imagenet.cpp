#include <ros/ros.h>
#include <jetson-inference/imageNet.h>

#include <jetson-utils/loadImage.h>
#include <jetson-utils/cudaFont.h>

#include <image_transport/image_transport.h>
#include "image_converter.h"

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace ros_deep_learning{

class ros_imagenet : public nodelet::Nodelet
{
    public:
        ~ros_imagenet()
        {
            ROS_INFO("\nshutting down...\n");
            delete net;
        }
        void onInit()
        {
            // get a private nodehandle
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            // get parameters from server, checking for errors as it goes
            std::string prototxt_path, model_path, mean_binary_path, class_labels_path;
            if(! private_nh.getParam("prototxt_path", prototxt_path) )
                ROS_ERROR("unable to read prototxt_path for imagenet node");
            if(! private_nh.getParam("model_path", model_path) )
                ROS_ERROR("unable to read model_path for imagenet node");
            if(! private_nh.getParam("class_labels_path", class_labels_path) )
                ROS_ERROR("unable to read class_labels_path for imagenet node");

            // make sure files exist (and we can read them)
            if( access(prototxt_path.c_str(),R_OK) )
                 ROS_ERROR("unable to read file \"%s\", check filename and permissions",prototxt_path.c_str());
            if( access(model_path.c_str(),R_OK) )
                 ROS_ERROR("unable to read file \"%s\", check filename and permissions",model_path.c_str());
            if( access(class_labels_path.c_str(),R_OK) )
                 ROS_ERROR("unable to read file \"%s\", check filename and permissions",class_labels_path.c_str());

            // create imageNet
            net = imageNet::Create(prototxt_path.c_str(),model_path.c_str(),NULL,class_labels_path.c_str());
            if( !net )
            {
                ROS_ERROR("imagenet:   failed to initialize imageNet model");
                return;
            }

		  // create image converter
		  imgCvt = new imageConverter();
	
		  if( !imgCvt )
		  {
			ROS_ERROR("failed to create imageConverter object");
			return;
		  }

            // setup image transport
            image_transport::ImageTransport it(private_nh);
            // subscriber for passing in images
            imsub = it.subscribe("imin", 10, &ros_imagenet::callback, this);
            // publisher for classifier output
            class_pub = private_nh.advertise<std_msgs::Int32>("class",10);
            // publisher for human-readable classifier output
            class_str_pub = private_nh.advertise<std_msgs::String>("class_str",10);
        }


    private:
        void callback(const sensor_msgs::ImageConstPtr& input)
        {
		  // convert the image to reside on GPU
		  if( !imgCvt || !imgCvt->Convert(input) )
		  {
			ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
			return;	
		  }

            float confidence = 0.0f;

            // classify image
            const int img_class = net->Classify(imgCvt->ImageGPU(), imgCvt->GetWidth(), imgCvt->GetHeight(), &confidence);

            // publish class
            std_msgs::Int32 class_msg;
            class_msg.data = img_class;
            class_pub.publish(class_msg);
            // publish class string
            std_msgs::String class_msg_str;
            class_msg_str.data = net->GetClassDesc(img_class);
            class_str_pub.publish(class_msg_str);

            if( img_class < 0 )
                ROS_ERROR("imagenet:  failed to classify (result=%i)", img_class);
        }

        // private variables
        image_transport::Subscriber imsub;
        imageConverter* imgCvt;
        ros::Publisher class_pub;
        ros::Publisher class_str_pub;
        imageNet* net;
};

#if ROS_VERSION_MINIMUM(1, 14, 0)
	PLUGINLIB_EXPORT_CLASS(ros_deep_learning::ros_imagenet, nodelet::Nodelet);
#else
	PLUGINLIB_DECLARE_CLASS(ros_deep_learning, ros_imagenet, ros_deep_learning::ros_imagenet, nodelet::Nodelet);
#endif

} // namespace ros_deep_learning
