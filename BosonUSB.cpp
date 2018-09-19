/*
-------------------------------------------------------------
-  FLIR Systems - Linux Boson  Capture & Recording          -
-------------------------------------------------------------
- Andres Prieto-Moreno           	        	    -
-------------------------------------------------------------
- Thanks to: https://jwhsmith.net/2014/12/capturing-a-webcam-stream-using-v4l2/
-------------------------------------------------------------

 BosonUSB [-r][-y][-z [2..4]][-s [bBv]][-f name][-d [num]]
	r	      : raw16 bits video input (default)
	y	      : agc-8 bits video input
	z [2..4]  : zoom factor
	f <name>  : record TIFFS in Folder <name>
	s [bBv]   : camera size : b=boson320, B=boson640, v=video 640x480
	d [num]   : linux video port

./BosonUSB               ->	opens Boson320 /dev/video0	 in RAW16 mode
./BosonUSB -r            ->	opens Boson320 /dev/video0	 in RAW16 mode
./BosonUSB -y            -> opens Boson320 /dev/video0	 in AGC-8bits mode
./BosonUSB -sB -d1       -> opens Boson640 /dev/video1  in RAW16 mode
./BosonUSB -sB -y -d2    -> opens Boson640 /dev/video2  in AGC-8bits mode
./BosonUSB -f cap        -> creates a folder named 'cap' and inside TIFF files (raw16, agc, yuv) will be located.

*/

#include <stdio.h>
#include <fcntl.h>			 // open, O_RDWR
#include <unistd.h>			 // close
#include <sys/ioctl.h>			 // ioctl
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>
#include <endian.h>

#include <opencv2/opencv.hpp>

#define VERSION_MAJOR 2
#define VERSION_MINOR 0

// Define COLOR CODES
#define RED	  "\x1B[31m"
#define GRN	  "\x1B[32m"
#define YEL	  "\x1B[33m"
#define BLU	  "\x1B[34m"
#define MAG	  "\x1B[35m"
#define CYN	  "\x1B[36m"
#define WHT	  "\x1B[37m"
#define RESET "\x1B[0m"

// Types of sensors supported
enum sensor_types {
	Boson320,
	Boson640,
	Video
};

/* ---------------------------- 16 bits Mode auxiliary functions ---------------------------------------*/

// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
void AGC_Basic_Linear(const cv::Mat &input_16, cv::Mat &output_8)
{
	// auxiliary variables for AGC calcultion
	uint16_t maxv = 0;
	uint16_t minv = 0xFFFF;
	uint16_t value;

	unsigned int value8;

	cv::Size size = input_16.size();
	int height = size.height;
	int width = size.width;

	// RUN a super basic AGC
	for (int i = 0; i < height; i++) {
		const uint16_t *p = input_16.ptr<uint16_t>(i);
		for (int j = 0; j < width; j++) {
			value = le16toh(p[j]);

			minv = std::min(minv, value);
			maxv = std::max(maxv, value);
		}
	}
	//printf("maxv=%04X, minv=%04X\n", maxv, minv);

	if (maxv - minv == 0) {
		return;
	}

	for (int i = 0; i < height; i++) {
		const uint16_t *p = input_16.ptr<uint16_t>(i);
		uint8_t *po = output_8.ptr<uint8_t>(i);
		for (int j = 0; j < width; j++) {
			value = le16toh(p[j]);
			value8 = (255 * (value - minv)) / (maxv - minv);
			// printf("%04X \n", value8);

			po[j] = value8 & 0xFF;
		}
	}
}

/* ---------------------------- Other Aux functions ---------------------------------------*/

// HELP INFORMATION
void print_help(void)
{
	printf(CYN "Boson Capture and Record Video tool v%i.%i" WHT "\n", VERSION_MAJOR, VERSION_MINOR);
	printf(CYN "FLIR Systems" WHT "\n\n");
	printf(WHT "use : " YEL "'BosonUSB -r' " WHT "to capture in raw-16 bits mode	  (default)\n");
	printf(WHT "Use : " YEL "'BosonUSB -y' " WHT "to capture in agc-8  bits mode\n");
	printf(WHT "Use : " YEL "'BosonUSB -z [1..4]'" WHT "Zoom   (default 1)\n");
	printf(WHT "Use : " YEL "'BosonUSB -f<name>' " WHT "record TIFFS in Folder <NAME>\n");
	printf(WHT "Use : " YEL "'BosonUSB -d<num>'	 " WHT "to open /dev/Video<num>  (default 0)\n");
	printf(WHT "Use : " YEL "'BosonUSB -s[b,B,v]'   " WHT "b=boson320, B=boson640 v=video (default 320)\n");
	printf(WHT "Press " YEL "'q' " WHT " to quit\n");
	printf("\n");
}

/* ---------------------------- Main Function ---------------------------------------*/
// ENTRY POINT
int main(int argc, char** argv)
{
	int fd;
	struct v4l2_capability cap;
	long frame = 0;	  // First frame number enumeration
	char video[32];	  // To store Video Port Device
	char label[128];  // To display the information

	const char * thermal_sensor_name;  // sensor name
	char filename[256];	// PATH/File_count
	char folder_name[256];  // To store the folder name

	// Default Program options
	bool raw_video = true;
	bool zoom_enable = false;
	int zoom_factor = 2;
	bool record_enable = false;

	int width, height;

	// To record images
	std::vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PXM_BINARY);

	// Video device by default
	const char *video_dev = "0";
	const char *folder;

	sensor_types my_thermal = Boson320;
	thermal_sensor_name ="Boson_320";

	int opt;
	while ((opt = getopt(argc, argv, "ryd:s:f:z::")) != -1) {
		switch(opt) {
		case 'r':
			raw_video = true;
			break;
		case 'y':
			raw_video = false;
			break;
		case 'd':
			video_dev = optarg;
			break;
		case 's':
			switch (*optarg) {
			case 'B':
				my_thermal = Boson640;
				thermal_sensor_name = "Boson_640";
				break;
			case 'v':
				my_thermal = Video;
				raw_video = false;
				thermal_sensor_name = "Video";
				break;
			case 'b':
				my_thermal = Boson320;
				thermal_sensor_name = "Boson_320";
				break;
			default:
				print_help();
				exit(1);
			}
			break;
		case 'f':
			record_enable = true;
			folder = optarg;
			break;
		case 'z':
			zoom_enable = true;
			if (optarg) {
				zoom_factor = atoi(optarg);
			}
			if (zoom_factor < 1 || zoom_factor > 4)
				zoom_factor = 1;
			break;
		default:
			print_help();
			exit(1);
		}
	}

	print_help();

	sprintf(video, "/dev/video%s", video_dev);

	// Folder name
	if (record_enable) {
		if (strlen(folder) <= 1) {	 // File name has to be more than two chars
			strcpy(folder_name, thermal_sensor_name);
		}
		if (mkdir(folder_name, 0700)) {
			perror(RED "mkdir" WHT);
			exit(1);
		}
		if (chdir(folder_name)) {
			perror(RED "chdir" WHT);
			exit(1);
		}
		printf(WHT ">>> Folder " YEL "%s" WHT " selected to record files\n", folder_name);
	}

	// Printf Sensor defined
	printf(WHT ">>> " YEL "%s" WHT " selected\n", thermal_sensor_name);

	// We open the Video Device
	printf(WHT ">>> " YEL "%s" WHT " selected\n", video);
	if ((fd = open(video, O_RDWR)) < 0){
		perror(RED "Error : OPEN. Invalid Video Device" WHT "\n");
		exit(1);
	}

	// Check VideoCapture mode is available
	if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0){
		perror(RED "ERROR : VIDIOC_QUERYCAP. Capabilities not available" WHT "\n");
		exit(1);
	}

	printf(WHT "\tDriver: %s\n\tCard: %s\n\tBus %s\n\tcapabilities: %08x\n",
		   cap.driver, cap.card, cap.bus_info, cap.capabilities);

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
		fprintf(stderr, RED "The device does not handle video capture." WHT "\n");
		exit(1);
	}

	struct v4l2_format format;

	// Two different FORMAT modes, 8 bits vs RAW16
	if (raw_video) {
		printf(WHT ">>> " YEL "16 bits " WHT "capture selected\n");

		// I am requiring thermal 16 bits mode
		format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;

		// Select the frame SIZE (will depend on the type of sensor)
		switch (my_thermal) {
		case Boson640:
			width = 640;
			height = 512;
			break;
		default:
		case Boson320:
			width = 320;
			height = 256;
			break;
		}
	}
	else { // 8- bits is always 640x512 (even for a Boson 320)
		printf(WHT ">>> " YEL "8 bits " WHT "YUV selected\n");
		format.fmt.pix.pixelformat = V4L2_PIX_FMT_YVU420; // thermal, works	  LUMA, full Cr, full Cb

		if (my_thermal == Video) {
			width = 640;
			height = 480;
		}
		else {
			width = 640;
			height = 512;
		}
	}

	// Common variables
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	struct v4l2_pix_format *pix = &format.fmt.pix;
	pix->width = width;
	pix->height = height;

	// request desired FORMAT
	if(ioctl(fd, VIDIOC_S_FMT, &format) < 0){
		perror(RED "VIDIOC_S_FMT" WHT);
		exit(1);
	}
	width = pix->width;
	height = pix->height;

	printf(WHT "\tPixel fmt: %c%c%c%c\n\tWidth: %d\n\tHeight %d\n",
		pix->pixelformat		 & 0xFF,
		(pix->pixelformat >>  8) & 0xFF,
		(pix->pixelformat >> 16) & 0xFF,
		(pix->pixelformat >> 24) & 0xFF,
		pix->width, pix->height);

	int conversion;
	int channels;
	int in_height = height;

	const char *img_fmt = "YUV";
	switch (pix->pixelformat){
	case V4L2_PIX_FMT_YUV420:
		in_height = height + height / 2;
		conversion = cv::COLOR_YUV2RGB_I420;
		channels = CV_8UC1;
		break;
	case V4L2_PIX_FMT_YUYV:
		conversion = CV_YUV2RGB_YVYU;
		channels = CV_8UC2;
		break;
	case V4L2_PIX_FMT_Y16:
		img_fmt = "RAW16 AGC";
		break;
	default:
		printf( "unknown format %d\n", pix->pixelformat);
		exit (1);
		break;
	}
	
	sprintf(label, "%s : %s", thermal_sensor_name, img_fmt);
	if (zoom_enable) {
		strcat(label, " Zoom");
	}

	// we need to inform the device about buffers to use.
	// and we need to allocate them.
	// we’ll use a single buffer, and map our memory using mmap.
	// All this information is sent using the VIDIOC_REQBUFS call and a
	// v4l2_requestbuffers structure:
	struct v4l2_requestbuffers bufrequest;
	bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufrequest.memory = V4L2_MEMORY_MMAP;
	bufrequest.count = 1;	// we are asking for one buffer

	if (ioctl(fd, VIDIOC_REQBUFS, &bufrequest) < 0){
		perror(RED "VIDIOC_REQBUFS" WHT);
		exit(1);
	}

	// Now that the device knows how to provide its data,
	// we need to ask it about the amount of memory it needs,
	// and allocate it. This information is retrieved using the VIDIOC_QUERYBUF call,
	// and its v4l2_buffer structure.

	struct v4l2_buffer bufferinfo;
	memset(&bufferinfo, 0, sizeof(bufferinfo));

	bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufferinfo.memory = V4L2_MEMORY_MMAP;
	bufferinfo.index = 0;

	if (ioctl(fd, VIDIOC_QUERYBUF, &bufferinfo) < 0){
		perror(RED "VIDIOC_QUERYBUF" WHT);
		exit(1);
	}

	// map fd+offset into a process location (kernel will decide due to our NULL). lenght and
	// properties are also passed
	printf(WHT ">>> Image width	 =" YEL "%i" WHT "\n", width);
	printf(WHT ">>> Image height =" YEL "%i" WHT "\n", height);
	printf(WHT ">>> Buffer lenght=" YEL "%i" WHT "\n", bufferinfo.length);

	void * buffer_start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE,MAP_SHARED, fd, bufferinfo.m.offset);

	if (buffer_start == MAP_FAILED){
		perror(RED "mmap" WHT);
		exit(1);
	}

	// Activate streaming
	int type = bufferinfo.type;
	if (ioctl(fd, VIDIOC_STREAMON, &type) < 0){
		perror(RED "VIDIOC_STREAMON" WHT);
		exit(1);
	}

	// Declarations for RAW16 representation
	// Will be used in case we are reading RAW16 format
	// Boson320 , Boson 640
	cv::Mat thermal16(height, width, CV_16U, buffer_start);	  // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode`
	cv::Mat thermal16_agc(height, width, CV_8U);		  // OpenCV output buffer : Data used to display the video

	// Declarations for Zoom representation
	// Will be used or not depending on program arguments
	cv::Size zoom_size(width * zoom_factor, height * zoom_factor);
	cv::Mat zoom_mat;

	cv::Mat in_mat(in_height, width, channels, buffer_start);  // OpenCV input buffer
	cv::Mat thermal_rgb(height, width, CV_8UC3);	   // OpenCV output buffer

	cv::Mat show_mat;

	// Read frame, do AGC, paint frame
	for (;;) {

		// Put the buffer in the incoming queue.
		if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0){
			perror(RED "VIDIOC_QBUF" WHT);
			exit(1);
		}

		// The buffer's waiting in the outgoing queue.
		if (ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
			perror(RED "VIDIOC_QBUF" WHT);
			exit(1);
		}

		if (raw_video) {
			// -----------------------------
			// RAW16 DATA
			AGC_Basic_Linear(thermal16, thermal16_agc);

			// Display thermal after 16-bits AGC... will display an image
			show_mat = thermal16_agc;

			if (record_enable) {
				sprintf(filename, "%s_raw16_%lu.tiff", thermal_sensor_name, frame);
				cv::imwrite(filename, thermal16 , compression_params);

				sprintf(filename, "%s_agc_%lu.tiff", thermal_sensor_name, frame);
				cv::imwrite(filename, thermal16_agc , compression_params);
			}
		}
		else {
			// ---------------------------------
			// DATA in YUV
			cv::cvtColor(in_mat, thermal_rgb, conversion, 0);
			show_mat = thermal_rgb;

			if (record_enable) {
				sprintf(filename, "%s_rgb_%lu.tiff", thermal_sensor_name, frame);
				cv::imwrite(filename, thermal_rgb, compression_params);
			}
		}

		if (zoom_enable) {
			resize(show_mat, zoom_mat, zoom_size);
			show_mat = zoom_mat;
		}

		cv::imshow(label, show_mat);

		// Press 'q' to exit
		if (cv::waitKey(1) == 'q') { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
			printf(WHT ">>> " RED "'q'" WHT " key pressed. Quitting !\n");
			break;
		}
		frame ++;
	}
	// Finish Loop . Exiting.

	// Deactivate streaming
	if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0){
		perror(RED "VIDIOC_STREAMOFF" WHT);
		exit(1);
	};

	close(fd);
	return EXIT_SUCCESS;
}
