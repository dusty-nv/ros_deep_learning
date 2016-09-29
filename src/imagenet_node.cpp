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
            delete net;
        }
        // onInit will make nodelet conversion easy
        void onInit(ros::NodeHandle& private_nh)
        {
            // create imageNet
            net = imageNet::Create();
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

            imgHeight = cv_im.rows;
            imgWidth = cv_im.cols;
            imgPixels = imgHeight*imgWidth;
            imgSize = imgPixels * sizeof(float4);
            float4* cpu_data = (float4*)(cv_im.data);

            // (failed cuda solution) register mapped memory
            //CUDA(cudaHostRegister(cpu_data, imgSize, cudaHostRegisterMapped));
            //float4* gpu_data;
            //CUDA(cudaHostGetDevicePointer(&gpu_data, cpu_data, cudaHostRegisterMapped));

            // (working cuda solution) allocate cuda memory
            CUDA(cudaMalloc(&gpu_data, imgSize));
            // copy to device
            CUDA(cudaMemcpy(gpu_data, cpu_data, imgSize, cudaMemcpyHostToDevice));
            // double check real quick...
            // float4* cpu_data2 = (float4*) malloc(imgPixels * sizeof(float4));
            // cudaMemcpy(cpu_data2, gpu_data, imgSize, cudaMemcpyDeviceToHost);
            // cv::Mat cv_im2(imgHeight, imgWidth, CV_32FC3,(void*) cpu_data2);
            // cv::imwrite("writeout2.png",cv_im2);
            // free(cpu_data2);

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

            //if( img_class < 0 )
            //    ROS_INFO("imagenet-console:  failed to classify (result=%i)\n", img_class);
            //else
            //{
            //    ROS_INFO("imagenet-console: image classified -> %2.5f%% class #%i (%s)\n",
            //            confidence * 100.0f, img_class, net->GetClassDesc(img_class));
            //}

            // (part of failed solution) unregister mapped memory
            // CUDA(cudaHostUnregister(cpu_data));

            // (part of working solution)
            CUDA(cudaFree(gpu_data));

        }

        // private variables
        image_transport::Subscriber imsub;
        ros::Publisher class_pub;
        ros::Publisher class_str_pub;
        imageNet* net;

        float4* gpu_data;

        uint32_t imgWidth;
        uint32_t imgHeight;
        uint32_t imgPixels;
        size_t   imgSize;

};

// main entry point
int main( int argc, char** argv )
{
    // init ros node
    ros::init(argc, argv, "imagenet");
    ros::NodeHandle n;

    ros_imagenet rin;

    rin.onInit(n);

    ros::spin();

    return 0;
}
