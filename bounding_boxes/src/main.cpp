#include "ros/ros.h"
#include "ros/time.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <pwd.h>
#include "highgui.h"
#include <ctime>
#include <boost/filesystem.hpp>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

#include "upper_body_detector/UpperBodyDetector.h"

using namespace std;
using namespace rapidjson;
using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;
using namespace upper_body_detector;

string createFolderStructure(); //just a forward declaration

const string node_name = "bounding_boxes";
bool pretty_json_print;
const string path = createFolderStructure();

inline string getHomeDir() {
	using namespace std;
	int myuid;
	passwd *mypasswd;
	myuid = getuid();
	mypasswd = getpwuid(myuid);
	return mypasswd->pw_dir;
}

inline void dateTime(string dt[2]) {
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[40];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, 40, "%d-%m-%Y", timeinfo);
	dt[0] = buffer;
	strftime(buffer, 40, "%H:%M:%S", timeinfo);
	dt[1] = buffer;
}

string createFolderStructure() {
	static bool DateDirCreated = false;
	string dt[2];
	dateTime(dt);
	string path = getHomeDir() + "/" + node_name + "/" + dt[0] + "/" + dt[1];

	if (DateDirCreated)
		return path;
	boost::filesystem::create_directories(path);
	DateDirCreated = true;

	ROS_DEBUG_STREAM(node_name<<": Will be writing files to:"<<path);

	return path;
}

void saveImg(const ImageConstPtr &img, bool rgb, string fname) {
	cv::Mat m;

	try {
		m = cv_bridge::toCvShare(img)->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	string suffix = rgb ? "_rgb" : "_depth";
	string ext = rgb ? ".bmp" : ".pgm";
	string h = path + "/" + fname + suffix + ext;
	cv::imwrite(h, m);
}

void jsonStr(const char* const s, Value& v, Document::AllocatorType& a) {
	char buffer[60];
	int len = sprintf(buffer, "%s", s);
	v.SetString(buffer, len, a);
	memset(buffer, 0, sizeof(buffer));
}

void saveJson(Document &d, string fname) {
	StringBuffer sbuffer;

	PrettyWriter<StringBuffer> writer(sbuffer); //change to instance of Writer to get smaller files (not very readable)

	d.Accept(writer);

	string json(sbuffer.GetString(), sbuffer.GetSize());
	string saveto = path + "/" + fname + ".json";
	ofstream of(saveto.c_str());
	of << json;
	if (!of.good())
		throw runtime_error("Can't write the JSON string to the file");
}

void saveOdomAndDetections(const Odometry::ConstPtr &odom, const upper_body_detector::UpperBodyDetector::ConstPtr &ubd,
		string fname) {

	Document d;
	d.SetObject();
	Document::AllocatorType& a = d.GetAllocator();

	Value header(kObjectType);
	header.AddMember("seq", odom->header.seq, a);

	Value stamp(kObjectType);
	stamp.AddMember("secs", odom->header.stamp.sec, a);
	stamp.AddMember("nsecs", odom->header.stamp.nsec, a);
	header.AddMember("stamp", stamp, a);

	Value v;
	jsonStr(odom->header.frame_id.c_str(), v, a);
	header.AddMember("frame_id", v, a);
	d.AddMember("header", header, a);
	//header done

	Value child_frame_id(kObjectType);
	Value v2;
	jsonStr(odom->child_frame_id.c_str(), v2, a);
	d.AddMember("child_frame_id", v2, a);
	//child_frame_id done

	Value pose(kObjectType);
	Value pose2(kObjectType);
	Value pos(kObjectType);
	pos.AddMember("x", odom->pose.pose.position.x, a);
	pos.AddMember("y", odom->pose.pose.position.y, a);
	pos.AddMember("z", odom->pose.pose.position.z, a);
	pose2.AddMember("position", pos, a);

	Value orient(kObjectType);
	orient.AddMember("x", odom->pose.pose.orientation.x, a);
	orient.AddMember("y", odom->pose.pose.orientation.y, a);
	orient.AddMember("z", odom->pose.pose.orientation.z, a);
	orient.AddMember("w", odom->pose.pose.orientation.w, a);
	pose2.AddMember("orientation", orient, a);
	pose.AddMember("pose", pose2, a);

	Value cov(kArrayType);
	for (int i = 0; i < odom->pose.covariance.size(); i++)
		cov.PushBack(odom->pose.covariance[i], a);

	pose.AddMember("covariance", cov, a);
	d.AddMember("pose", pose, a);
	//pose done

	Value twist(kObjectType);
	Value twist2(kObjectType);
	Value linear(kObjectType);
	linear.AddMember("x", odom->twist.twist.linear.x, a);
	linear.AddMember("y", odom->twist.twist.linear.y, a);
	linear.AddMember("z", odom->twist.twist.linear.z, a);
	twist2.AddMember("linear", linear, a);

	Value angular(kObjectType);
	angular.AddMember("x", odom->twist.twist.angular.x, a);
	angular.AddMember("y", odom->twist.twist.angular.y, a);
	angular.AddMember("z", odom->twist.twist.angular.z, a);
	twist2.AddMember("angular", angular, a);
	twist.AddMember("twist", twist2, a);

	Value cov2(kArrayType);
	for (int i = 0; i < odom->twist.covariance.size(); i++)
		cov2.PushBack(odom->twist.covariance[i], a);

	twist.AddMember("covariance", cov2, a);
	d.AddMember("twist", twist, a);
	//twist done
	//entire odom msg done

	Value detectionsx(kArrayType);
	for (int i = 0; i < ubd->pos_x.size(); i++)
		detectionsx.PushBack(ubd->pos_x[i], a);

	Value detectionsy(kArrayType);
	for (int i = 0; i < ubd->pos_y.size(); i++)
		detectionsy.PushBack(ubd->pos_y[i], a);

	Value detectionsw(kArrayType);
	for (int i = 0; i < ubd->width.size(); i++)
		detectionsw.PushBack(ubd->width[i], a);

	Value detectionsh(kArrayType);
	for (int i = 0; i < ubd->height.size(); i++)
		detectionsh.PushBack(ubd->height[i], a);

	d.AddMember("pos_x", detectionsx, a);
	d.AddMember("pos_y", detectionsy, a);
	d.AddMember("width", detectionsw, a);
	d.AddMember("height", detectionsh, a);

	//ubd message done

	saveJson(d, fname);

}

void callback(const ImageConstPtr &depth, const ImageConstPtr &color,
		const upper_body_detector::UpperBodyDetector::ConstPtr &ubd, const Odometry::ConstPtr &odom) {
	if (ubd->pos_x.empty()) {
		return;
	}

	ostringstream stream;
	stream << ros::Time::now();
	string fname = stream.str();

	saveImg(color, true, fname);
	saveImg(depth, false, fname);
	saveOdomAndDetections(odom, ubd, fname + "_odom_detect");
}

bool checkParam(bool success, string param) {
	if (!success) {
		ROS_FATAL(
				"Parameter: '%s' could not be found! Please make sure that the parameters are available on the parameter server or start with 'load_params_from_file:=true'",
				param.c_str());
	}
	return success;
}

void camInfo2Json(Document &d, sensor_msgs::CameraInfoConstPtr &caminfo, string prefix) {

	Document::AllocatorType& a = d.GetAllocator();

	Value pref;
	jsonStr(prefix.c_str(), pref, a);
	d.AddMember("prefix", pref, a);
	//prefix done
	Value header(kObjectType);
	header.AddMember("seq", caminfo->header.seq, a);

	Value stamp(kObjectType);
	stamp.AddMember("secs", caminfo->header.stamp.sec, a);
	stamp.AddMember("nsecs", caminfo->header.stamp.nsec, a);
	header.AddMember("stamp", stamp, a);

	Value v;
	jsonStr(caminfo->header.frame_id.c_str(), v, a);
	header.AddMember("frame_id", v, a);
	d.AddMember("header", header, a);
	//header done

	Value height(kObjectType);
	height.AddMember("height", caminfo->height, a);
	d.AddMember("height", height, a);
	Value width(kObjectType);
	width.AddMember("width", caminfo->width, a);
	d.AddMember("width", width, a);
	//width and height done

	Value child_frame_id(kObjectType);
	Value v2;
	jsonStr(caminfo->distortion_model.c_str(), v2, a);
	d.AddMember("distortion_model", v2, a);
	// distortion_model done

	Value distD(kArrayType);
	for (int i = 0; i < caminfo->D.size(); i++)
		distD.PushBack(caminfo->D[i], a);
	d.AddMember("D", distD, a);

	Value distK(kArrayType);
	for (int i = 0; i < caminfo->K.size(); i++)
		distK.PushBack(caminfo->K[i], a);
	d.AddMember("K", distK, a);

	Value distR(kArrayType);
	for (int i = 0; i < caminfo->R.size(); i++)
		distR.PushBack(caminfo->R[i], a);
	d.AddMember("R", distR, a);

	Value distP(kArrayType);
	for (int i = 0; i < caminfo->P.size(); i++)
		distP.PushBack(caminfo->P[i], a);
	d.AddMember("P", distP, a);
	// D, K, R, P done

	Value binning_x(kObjectType);
	binning_x.AddMember("binning_x", caminfo->binning_x, a);
	d.AddMember("binning_x", binning_x, a);
	Value binning_y(kObjectType);
	binning_y.AddMember("binning_y", caminfo->binning_y, a);
	d.AddMember("binning_y", binning_y, a);
	// binning x and y done

	Value roi(kObjectType);
	roi.AddMember("x_offset", caminfo->roi.x_offset, a);
	roi.AddMember("y_offset", caminfo->roi.y_offset, a);
	roi.AddMember("height", caminfo->roi.height, a);
	roi.AddMember("width", caminfo->roi.width, a);
	roi.AddMember("do_rectify", caminfo->roi.do_rectify ? true : false, a);
	d.AddMember("roi", roi, a);
	//roi done
}

void saveCamInfo(sensor_msgs::CameraInfoConstPtr rgbInfo, sensor_msgs::CameraInfoConstPtr depthInfo) {

	Document d;
	d.SetObject();

	camInfo2Json(d, rgbInfo, "RGB_INFO");
	camInfo2Json(d, depthInfo, "DEPTH_INFO");
	saveJson(d, "00_camera_info");
}

int main(int argc, char **argv) {
	// Set up ROS.
	ros::init(argc, argv, node_name);
	ros::NodeHandle n;

	// Declare variables that can be modified by launch file or command line.
	int queue_size;
	string cam_ns;
	string topic_gp;
	string pub_markers_topic;
	string topic_ubd;
	string topic_odom;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param("queue_size", queue_size, int(5));
	private_node_handle_.param("camera_namespace", cam_ns, string("/head_xtion"));
	private_node_handle_.param("upper_body_detector", topic_ubd, string("/upper_body_detector/detections"));
	private_node_handle_.param("odom", topic_odom, string("/odom"));

	string topic_color_image;
	private_node_handle_.param("rgb_image", topic_color_image, string("/rgb/image_raw"));
	topic_color_image = cam_ns + topic_color_image;
	string topic_camera_info_depth;
	private_node_handle_.param("camera_info_depth", topic_camera_info_depth, string("/depth_registered/camera_info"));
	topic_camera_info_depth = cam_ns + topic_camera_info_depth;
	string topic_camera_info_rgb;
	private_node_handle_.param("camera_info_rgb", topic_camera_info_rgb, string("/rgb/camera_info"));
	topic_camera_info_rgb = cam_ns + topic_camera_info_rgb;
	string topic_depth_image;
	private_node_handle_.param("depth_image", topic_depth_image, string("/depth_registered/image_rect"));
	topic_depth_image = cam_ns + topic_depth_image;
	private_node_handle_.param("pretty_json", pretty_json_print, false);

	// Printing queue size
	ROS_DEBUG_STREAM(node_name<<": Queue size for synchronisation is set to:"<<queue_size);

	// Image transport handle
	image_transport::ImageTransport it(private_node_handle_);

	// Create a subscriber.
	// Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
	image_transport::SubscriberFilter subscriber_depth;
	subscriber_depth.subscribe(it, topic_depth_image.c_str(), 1);
	message_filters::Subscriber<CameraInfo> subscriber_camera_info(n, topic_camera_info_depth.c_str(), 1);
	image_transport::SubscriberFilter subscriber_color;
	subscriber_color.subscribe(it, topic_color_image.c_str(), 1);
	message_filters::Subscriber<upper_body_detector::UpperBodyDetector> subscriber_ubd(n, topic_ubd.c_str(), 1);
	message_filters::Subscriber<Odometry> subscriber_odom(n, topic_odom.c_str(), 1);

	//The real queue size for synchronisation is set here.
	sync_policies::ApproximateTime<Image, Image, upper_body_detector::UpperBodyDetector, Odometry> MySyncPolicy(
			queue_size);
	MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.

	// Create synchronization policy. Here: async because time stamps will never match exactly
	const sync_policies::ApproximateTime<Image, Image, upper_body_detector::UpperBodyDetector, Odometry> MyConstSyncPolicy =
			MySyncPolicy;
	Synchronizer<sync_policies::ApproximateTime<Image, Image, upper_body_detector::UpperBodyDetector, Odometry> > sync(
			MyConstSyncPolicy, subscriber_depth, subscriber_color, subscriber_ubd, subscriber_odom);
	// Register one callback for all topics
	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

	sensor_msgs::CameraInfoConstPtr depthInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
			topic_camera_info_depth);
	sensor_msgs::CameraInfoConstPtr rgbInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
			topic_camera_info_rgb);

	saveCamInfo(rgbInfo, depthInfo);

	ros::spin();

	return 0;
}

