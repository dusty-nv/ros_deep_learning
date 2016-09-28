#include <ros/ros.h>
#include <jetson-inference/imageNet.h>

#include <jetson-inference/loadImage.h>
#include <jetson-inference/cudaFont.h>

// main entry point
int main( int argc, char** argv )
{
	ROS_INFO("imagenet-console\n  args (%i):  ", argc);

	for( int i=0; i < argc; i++ )
		ROS_INFO("%i [%s]  ", i, argv[i]);

	ROS_INFO("\n\n");


	// retrieve filename argument
	if( argc < 2 )
	{
		ROS_INFO("imagenet-console:   input image filename required\n");
		return 0;
	}

	const char* imgFilename = argv[1];

	// create imageNet
	imageNet* net = imageNet::Create();

	if( !net )
	{
		ROS_INFO("imagenet-console:   failed to initialize imageNet\n");
		return 0;
	}

	// load image from file on disk
	float* imgCPU    = NULL;
	float* imgCUDA   = NULL;
	int    imgWidth  = 0;
	int    imgHeight = 0;

	if( !loadImageRGBA(imgFilename, (float4**)&imgCPU, (float4**)&imgCUDA, &imgWidth, &imgHeight) )
	{
		ROS_INFO("failed to load image '%s'\n", imgFilename);
		return 0;
	}

	float confidence = 0.0f;

	// classify image
	const int img_class = net->Classify(imgCUDA, imgWidth, imgHeight, &confidence);

	if( img_class < 0 )
		ROS_INFO("imagenet-console:  failed to classify '%s'  (result=%i)\n", imgFilename, img_class);
	else
	{
		ROS_INFO("imagenet-console:  '%s' -> %2.5f%% class #%i (%s)\n", imgFilename, confidence * 100.0f, img_class, net->GetClassDesc(img_class));

		if( argc > 2 )
		{
			const char* outputFilename = argv[2];

			cudaFont* font = cudaFont::Create();

			if( font != NULL )
			{
				char str[512];
				ROS_INFO(str, "%2.5f%% %s", confidence * 100.0f, net->GetClassDesc(img_class));
				font->RenderOverlay((float4*)imgCUDA, (float4*)imgCUDA, imgWidth, imgHeight, (const char*)str, 10, 10);
			}

			ROS_INFO("imagenet-console:  attempting to save output image to '%s'\n", outputFilename);

			if( !saveImageRGBA(outputFilename, (float4*)imgCPU, imgWidth, imgHeight) )
				ROS_INFO("imagenet-console:  failed to save output image to '%s'\n", outputFilename);
			else
				ROS_INFO("imagenet-console:  completed saving '%s'\n", outputFilename);
		}
	}

	ROS_INFO("\nshutting down...\n");
	CUDA(cudaFreeHost(imgCPU));
	delete net;

    return 0;
}
