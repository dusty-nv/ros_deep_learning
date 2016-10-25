#include <ros/ros.h>
#include <jetson-inference/imageNet.h>

#include <jetson-inference/loadImage.h>
#include <jetson-inference/cudaFont.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>


class ros_imagenet
{
    public:
        ~ros_imagenet()
        {
            ROS_INFO("\nshutting down...\n");
            if(gpu_data)
                CUDA(cudaFree(gpu_data));
            delete net;
        }
        // onInit will make nodelet conversion easy
        void onInit(ros::NodeHandle& private_nh)
        {
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
                ROS_INFO("imagenet-console:   failed to initialize imageNet\n");
                return;
            }

            // subscribe to image feed on imin
            ROS_INFO("Ryan, uncomment the line under this message when you switch to nodelet operation");
            //ros::NodeHandle& private_nh = getPrivateNodeHandle();
            image_transport::ImageTransport it(private_nh);
            // subscriber for passing in images
            imsub = it.subscribe("imin", 10, &ros_imagenet::callback, this);
            // publisher for classifier output
            class_pub = private_nh.advertise<std_msgs::Int32>("class",10);
            // publisher for human-readable classifier output
            class_str_pub = private_nh.advertise<std_msgs::String>("class_str",10);

            // init gpu memory
            gpu_data = NULL;
        }


    private:
        void callback(const sensor_msgs::ImageConstPtr& input)
        {
            cv::Mat cv_im = cv_bridge::toCvCopy(input, "bgr8")->image;
            ROS_INFO("image ptr at %p",cv_im.data);
            cv::imwrite("writeout.png",cv_im);
            // convert bit depth
            cv_im.convertTo(cv_im,CV_32FC3);
            // convert color
            cv::cvtColor(cv_im,cv_im,CV_BGR2RGBA);

            // allocate GPU data if necessary
            if(!gpu_data){
                ROS_INFO("first allocation");
                CUDA(cudaMalloc(&gpu_data, cv_im.rows*cv_im.cols * sizeof(float4)));
            }else if(imgHeight != cv_im.rows || imgWidth != cv_im.cols){
                ROS_INFO("re allocation");
                // reallocate for a new image size if necessary
                CUDA(cudaFree(gpu_data));
                CUDA(cudaMalloc(&gpu_data, cv_im.rows*cv_im.cols * sizeof(float4)));
            }

            imgHeight = cv_im.rows;
            imgWidth = cv_im.cols;
            imgSize = cv_im.rows*cv_im.cols * sizeof(float4);
            float4* cpu_data = (float4*)(cv_im.data);

            // copy to device
            CUDA(cudaMemcpy(gpu_data, cpu_data, imgSize, cudaMemcpyHostToDevice));

            float confidence = 0.0f;

            // classify image
            const int img_class = net->Classify((float*)gpu_data, imgWidth, imgHeight, &confidence);

            // publish class
            std_msgs::Int32 class_msg;
            class_msg.data = img_class;
            class_pub.publish(class_msg);
            // publish class string
            std_msgs::String class_msg_str;
            class_msg_str.data = net->GetClassDesc(img_class);
            class_str_pub.publish(class_msg_str);

            if( img_class < 0 )
                ROS_INFO("imagenet-console:  failed to classify (result=%i)\n", img_class);

        }

        // private variables
        image_transport::Subscriber imsub;
        ros::Publisher class_pub;
        ros::Publisher class_str_pub;
        imageNet* net;

        float4* gpu_data;

        uint32_t imgWidth;
        uint32_t imgHeight;
        size_t   imgSize;

};

// main entry point
int main( int argc, char** argv )
{
    // init ros node
    ros::init(argc, argv, "imagenet_node");
    ros::NodeHandle n("~");

    ros_imagenet rin;

    rin.onInit(n);

    ros::spin();

    return 0;
}
