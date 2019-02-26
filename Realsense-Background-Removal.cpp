#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

int main()
{
	rs2::pipeline pipe;
	rs2::colorizer color_map;
	rs2::pipeline_profile profile;
	rs2::device selected_device;

	rs2_stream align_to;
	rs2::frameset frame_data;

	char* window_name = "Display";
	int thresh_value = 190;

	Mat color_frame;
	Mat depth_frame;
	Mat depth_mask;
	Mat final_frame;

	profile = pipe.start();
	selected_device = profile.get_device();
	rs2::depth_sensor sensor = selected_device.first<rs2::depth_sensor>(); // TO DO: Better device checking & acquiring

	namedWindow(window_name, CV_WINDOW_AUTOSIZE);

	sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, 3);
	color_map.set_option(RS2_OPTION_VISUAL_PRESET, 1.f); // Lock depth scale
	color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f); // White to black visual scheme

	align_to = RS2_STREAM_COLOR;
	rs2::align align(align_to); // TO DO: Ensure color stream is accessible before setting

	cvCreateTrackbar("Depth:", window_name, &thresh_value, 255);
	
	while (waitKey(1) < 0)
	{
		frame_data = pipe.wait_for_frames();
		auto processed = align.process(frame_data);

		rs2::video_frame aligned_color = processed.first(align_to);
		rs2::depth_frame aligned_depth = color_map(processed.get_depth_frame());

		color_frame = Mat(
			Size(
				aligned_color.get_width(),
				aligned_color.get_height()
			),
			CV_8UC3, // RGB8
			(void*)aligned_color.get_data(),
			Mat::AUTO_STEP
		);

		depth_frame = Mat(
			Size(
				aligned_depth.get_width(),
				aligned_depth.get_height()
			),
			CV_8UC3, // RGB8
			(void*)aligned_depth.get_data(),
			Mat::AUTO_STEP
		);

		depth_mask = Mat(
			Size(
				aligned_depth.get_width(),
				aligned_depth.get_height()
			),
			CV_8UC1 // Gray
		);

		final_frame = Mat(
			Size(
				aligned_depth.get_width(),
				aligned_depth.get_height()
			),
			CV_8UC4 // RGBA
		);

		// Reorder Realsense RGB camera
		cvtColor(color_frame, color_frame, CV_BGR2RGB);
		
		if (thresh_value)
		{
			threshold(depth_frame, depth_mask, thresh_value, 255, THRESH_BINARY);
		}

		// Morphological transformations here, closing

		color_frame.copyTo(final_frame, depth_mask); // Apply binary mask

		// TO DO: Display png, soft edge alpha
		// Possible NDI/Spout2 output?

		imshow(window_name, final_frame);
		imshow("Depth Mask", depth_mask);
	}
	return EXIT_SUCCESS;
}