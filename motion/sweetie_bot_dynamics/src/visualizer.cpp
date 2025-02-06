#include <xmlrpcpp/XmlRpcException.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>

#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <tf_conversions/tf_kdl.h>

#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sweetie_bot_kinematics_msgs/JointStateAccel.h>
#include <sweetie_bot_kinematics_msgs/SupportState.h>
#include <sweetie_bot_kinematics_msgs/RigidBodyState.h>
#include <sweetie_bot_kinematics_msgs/BalanceState.h>

#include <sweetie_bot_orocos_misc/joint_state_check.hpp>
#include <sweetie_bot_orocos_misc/message_checks.hpp>

struct ColorRGBAInit : public std_msgs::ColorRGBA
{
	public:
		ColorRGBAInit() : std_msgs::ColorRGBA() {}
		//ColorRGBAInit(const ColorRGBAInit& color) : ColorRGBA(color) {}
		ColorRGBAInit(_r_type _r, _g_type _g, _b_type _b, _a_type _a = 1.0) { r = _r; b = _b; g = _g; a = _a; }
};


class DynamicsVisualizer 
{
	protected:
		typedef sensor_msgs::JointState JointState;
		typedef sweetie_bot_kinematics_msgs::JointStateAccel JointStateAccel;
		typedef sweetie_bot_kinematics_msgs::SupportState SupportState;
		typedef sweetie_bot_kinematics_msgs::RigidBodyState RigidBodyState;
		typedef sweetie_bot_kinematics_msgs::BalanceState BalanceState;
		typedef visualization_msgs::MarkerArray MarkerArray;
		typedef visualization_msgs::Marker Marker;

	protected:
		static const ColorRGBAInit RED;
		static const ColorRGBAInit GREEN;
		static const ColorRGBAInit MAGENTA;
		static const ColorRGBAInit BLUE;
		static const ColorRGBAInit LIGHT_BLUE;
		static const ColorRGBAInit YELLOW;

	protected:
		// NODE INTERFACE
		// NodeHandler
		ros::NodeHandle node_handler;
		// publisers
		ros::Publisher joints_pub;
		ros::Publisher markers_pub;
		// subscribers
		ros::Subscriber joints_accel_sub;
		ros::Subscriber supports_sub;
		ros::Subscriber wrenches_sub;
		ros::Subscriber base_sub;
		ros::Subscriber balance_sub;
		// tf
		tf::TransformListener tf_listener;
		// timer
		ros::Timer timer;

		// PARAMETERS
		std::string robot_model_ns_param;
		double point_size_param;
		double torque_scale_param;
		double force_scale_param;
		double velocity_angular_scale_param;
		bool display_twist;
		int balance_history_length;

		// robot_model parameters cache
		std::map<std::string, KDL::Vector> contact_points_cache;

		// BUFFERS
		SupportState supports;
		RigidBodyState wrenches;
		RigidBodyState base;
		BalanceState balance;

		// VIZUALIZATION MARKERS
		MarkerArray marker_balance;

	public:
		DynamicsVisualizer()
		{
			// input
			joints_accel_sub = node_handler.subscribe<JointStateAccel>("joint_state_accel", 1, &DynamicsVisualizer::callbackJointsAccelSub, this);
			base_sub = node_handler.subscribe<RigidBodyState>("base", 1, &DynamicsVisualizer::callbackBaseSub, this);
			wrenches_sub = node_handler.subscribe<RigidBodyState>("wrenches", 1, &DynamicsVisualizer::callbackWrenchesSub, this);
			supports_sub = node_handler.subscribe<SupportState>("supports", 1, &DynamicsVisualizer::callbackSupportsSub, this);
			balance_sub = node_handler.subscribe<BalanceState>("balance", 1, &DynamicsVisualizer::callbackBalanceSub, this);
			// output
			markers_pub = node_handler.advertise<MarkerArray>("marker_array", 3);
			joints_pub = node_handler.advertise<JointState>("joint_efforts", 1);	

			// get node parameters
			// TODO: type and value checks
			if (!ros::param::get("~robot_model_namespace", robot_model_ns_param)) {
				robot_model_ns_param = "";
			}
			if (!ros::param::get("~point_size", point_size_param)) {
				point_size_param = 0.005;
			}
			if (!ros::param::get("~torque_scale", torque_scale_param)) {
				torque_scale_param = 0.01;
			}
			if (!ros::param::get("~force_scale", force_scale_param)) {
				force_scale_param = 0.01;
			}
			if (!ros::param::get("~display_twist", display_twist)) {
				display_twist = true;
			}
			if (!ros::param::get("~velocity_angular_scale", velocity_angular_scale_param)) {
				velocity_angular_scale_param = 1.0/(2.0*M_PI);
			}
			if (!ros::param::get("~balance_history_length", balance_history_length)) {
				balance_history_length = 1;
			}

			// maker array buffer buffers
			prepareBalanceBuffers();

			// timer
			timer = node_handler.createTimer(ros::Duration(0.05), &DynamicsVisualizer::loop, this);
			timer.start();

			ROS_INFO("SweeterBot dynamics visualizer started!");
		}


		std::string getContactFrame(const std::string& name) {
			std::string frame;
			if (node_handler.getParamCached(robot_model_ns_param + "/robot_model/chains/" + name + "/last_link", frame)) {
				return frame;
			}
			else throw ros::Exception("Unable to determine last_link frame name of " + name + " kinematic chain. Check if robot_model is loaded onto Parameter Service in namespace '" + robot_model_ns_param + "'.");
		}

		KDL::Vector getContactPoint(const std::string& name) {
			// try to find parameter in cache
			auto it = contact_points_cache.find(name);
			if (it != contact_points_cache.end()) return it->second;
			// contact not found. Request it from robot model

			try { 
				XmlRpc::XmlRpcValue points_param;
				node_handler.getParam(robot_model_ns_param + "/robot_model/contacts/" + name + "/points", points_param);
				// depend on OROCOS version arrays have different representations
				const XmlRpc::XmlRpcValue& point_param = (points_param.getType() == XmlRpc::XmlRpcValue::TypeStruct) ? points_param["Element0"] : points_param[0]; 
				KDL::Vector point(point_param["X"], point_param["Y"], point_param["Z"]);
				// cache up retrived point
				contact_points_cache[name] = point;
				return point;
			}
			catch (const XmlRpc::XmlRpcException& e) {
				throw ros::Exception("Unable to get contact point " + name + ". Check if robot_model is loaded into Parameter Service and contact point exists.");
			}
		}

		void callbackJointsAccelSub(const JointStateAccel::ConstPtr& msg)
		{
			// copy to joints state 
			JointState joints;
			joints.header = msg->header;
			joints.name = msg->name;
			joints.position = msg->position;
			joints.velocity = msg->velocity;
			joints.effort = msg->effort;
			//TODO typestamp? 
			// publish
			joints_pub.publish(joints);
		}

		void callbackSupportsSub(const SupportState::ConstPtr& msg) 
		{
			if (sweetie_bot::isValidSupportStateNameSuppCont(*msg)) {
				// buffer message for following vizualization
				supports = *msg;
			}
		}

		void callbackWrenchesSub(const RigidBodyState::ConstPtr& msg) 
		{
			if (sweetie_bot::isValidRigidBodyStateNameFrame(*msg)) {
				// buffer message for following vizualization
				wrenches = *msg;
			}
		}

		void callbackBaseSub(const RigidBodyState::ConstPtr& msg) 
		{
			if (sweetie_bot::isValidRigidBodyStateNameFrame(*msg, 1)) {
				// buffer message for following vizualization
				base = *msg;
			}
		}

		void callbackBalanceSub(const BalanceState::ConstPtr& msg) 
		{
			balance = *msg;
		}

		void loop(const ros::TimerEvent&) 
		{
			if (supports.name.size() > 0) visualizeSupports();
			if (wrenches.name.size() > 0) visualizeWrenches();
			visualizeBalance();
		}

		void visualizeSupports() 
		{
			// prepare Markers message
			MarkerArray marker_array;

			// add contact points
			marker_array.markers.resize(1);
			Marker& marker_points = marker_array.markers[0];
			// message with points
			marker_points.header.frame_id = "odom_combined"; // contact points are displayed in world frame
			marker_points.header.stamp = ros::Time::now();
			marker_points.ns = "contacts";
			marker_points.id = 0;
			marker_points.type = visualization_msgs::Marker::POINTS;
			marker_points.action = 0; // add/modify 
			marker_points.pose.orientation.w = 1.0; // other elements are zeros
			marker_points.scale.x = point_size_param; marker_points.scale.y = point_size_param; marker_points.scale.z = 0.0;
			marker_points.color = RED;
			marker_points.lifetime = ros::Duration(1.0); // one second
			marker_points.frame_locked = true; // odom_combined is fixed frame

			for(int k = 0; k < supports.name.size(); k++) {
				if (supports.contact[k] == "") continue;
				try {
					geometry_msgs::PointStamped point_stamped;
					// get point information
					point_stamped.header.frame_id = getContactFrame(supports.name[k]);
					tf::pointKDLToMsg(getContactPoint(supports.contact[k]), point_stamped.point);
					// convert to world frame
					tf_listener.transformPoint("odom_combined", point_stamped, point_stamped); 
					// add point marker
					marker_points.points.push_back(point_stamped.point);
					// check if point support 
					if (supports.support[k] > 0.0) {
						// set color: point is in contact
						marker_points.colors.push_back(RED);
					}
					else {
						// set color: point is not in contact
						marker_points.colors.push_back(GREEN);
					}
				}
				catch (tf::TransformException& e) {
					ROS_ERROR("tf error: %s", e.what());
					return;
				}
				catch (ros::Exception& e) { // getContactFrame or getContactPoint failed
					ROS_ERROR("ROS error: %s", e.what());
				}
			}
			// publish resulting message
			markers_pub.publish(marker_array);
		}

		void visualizeWrenches() 
		{
			// prepare Markers message
			MarkerArray marker_array;
			// add contact points
			marker_array.markers.resize(1);
			Marker& marker = marker_array.markers[0];
			// message contact forces vizualization
			marker.header.frame_id = "odom_combined"; // forces are supplied in base_link frame
			marker.header.stamp = ros::Time::now();
			marker.ns = "limbs_external_forces_and_twists";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::LINE_LIST;
			marker.action = 0; // add/modify 
			marker.pose.orientation.w = 1.0; // other elements are zeros
			marker.scale.x = point_size_param/2; marker.scale.y = 0.0; marker.scale.z = 0.0;
			marker.color = RED;
			marker.lifetime = ros::Duration(1.0); // one second
			marker.frame_locked = true; // odom_combined is fixed frame

			// RigidBodyState messge contains all objects in world frame coordinates
			for(int k = 0; k < wrenches.wrench.size(); k++) {
					// get wrench and bring it to limb tip
					KDL::Wrench wrench = wrenches.wrench[k];
					KDL::Vector point = wrenches.frame[k].p;
					wrench = wrench.RefPoint(point);
					// wrench visualization
					KDL::Vector point_force = point + wrench.force*force_scale_param; // force visualization
					KDL::Vector point_torque = point + wrench.torque*torque_scale_param; // torque visualization
					// add to line list
					marker.points.emplace_back(); tf::pointKDLToMsg(point, marker.points.back()); marker.colors.push_back(RED);
					marker.points.emplace_back(); tf::pointKDLToMsg(point_force, marker.points.back()); marker.colors.push_back(RED);
					marker.points.emplace_back(); tf::pointKDLToMsg(point, marker.points.back()); marker.colors.push_back(BLUE);
					marker.points.emplace_back(); tf::pointKDLToMsg(point_torque, marker.points.back()); marker.colors.push_back(BLUE);
			}
			if (display_twist) {
				// display limbs twists
				for(int k = 0; k < wrenches.twist.size(); k++) {
					// get velocity and bring it to limb tip
					KDL::Twist twist = wrenches.twist[k];
					KDL::Vector point = wrenches.frame[k].p;
					twist = twist.RefPoint(point);
					// velocity visualization
					KDL::Vector point_vel = point + twist.vel; // linear vel visualization
					KDL::Vector point_rot = point + twist.rot*velocity_angular_scale_param; // angular vel visualization
					// add to line list
					marker.points.emplace_back(); tf::pointKDLToMsg(point, marker.points.back()); marker.colors.push_back(GREEN);
					marker.points.emplace_back(); tf::pointKDLToMsg(point_vel, marker.points.back()); marker.colors.push_back(GREEN);
					marker.points.emplace_back(); tf::pointKDLToMsg(point, marker.points.back()); marker.colors.push_back(YELLOW);
					marker.points.emplace_back(); tf::pointKDLToMsg(point_rot, marker.points.back()); marker.colors.push_back(YELLOW);
				}
				// display base twist
				if (base.name.size() == 1) {
					KDL::Twist twist = base.twist[0];
					KDL::Vector point = base.frame[0].p;
					twist = twist.RefPoint(point);
					// velocity visualization
					KDL::Vector point_vel = point + twist.vel; // linear vel visualization
					KDL::Vector point_rot = point + twist.rot*velocity_angular_scale_param; // angular vel visualization
					// add to line list
					marker.points.emplace_back(); tf::pointKDLToMsg(point, marker.points.back()); marker.colors.push_back(GREEN);
					marker.points.emplace_back(); tf::pointKDLToMsg(point_vel, marker.points.back()); marker.colors.push_back(GREEN);
					marker.points.emplace_back(); tf::pointKDLToMsg(point, marker.points.back()); marker.colors.push_back(YELLOW);
					marker.points.emplace_back(); tf::pointKDLToMsg(point_rot, marker.points.back()); marker.colors.push_back(YELLOW);
				}
			}
			// publish resulting message
			markers_pub.publish(marker_array);
		}

		void prepareBalanceBuffers() 
		{
			marker_balance.markers.resize(6);

			Marker marker;
			Marker& marker_points = marker_balance.markers[0];
			Marker& marker_lines = marker_balance.markers[1];
			Marker& marker_zmp = marker_balance.markers[2];
			Marker& marker_zmp_pt = marker_balance.markers[3];
			Marker& marker_cog = marker_balance.markers[4];
			Marker& marker_cog_pt = marker_balance.markers[5];

			// common header
			marker.header.frame_id = "odom_combined"; // we can calculate ZMP only on world frame
			marker.header.stamp = ros::Time::now();
			marker.ns = "balance";
			marker.action = 0; // add/modify 
			marker.pose.orientation.w = 1.0; // other elements are zeros
			marker.color = GREEN;
			marker.lifetime = ros::Duration(1.0); // one second
			marker.frame_locked = true; // odom_combined IS FIXED FRAMe

			// message with CoM and ZMP points
			marker_points = marker;
			marker_points.id = 0;
			marker_points.type = visualization_msgs::Marker::POINTS;
			marker_points.scale.x = point_size_param; marker_points.scale.y = point_size_param; marker_points.scale.z = 0.0;
			marker_points.points.resize(4);
			marker_points.colors = { GREEN, MAGENTA, LIGHT_BLUE, LIGHT_BLUE }; // CoP, ZMP, CoM projection, CoM
			// message with support polygone
			marker_lines = marker;
			marker_lines.id = 1;
			marker_lines.type = visualization_msgs::Marker::LINE_STRIP;
			marker_lines.scale.x = point_size_param/2; marker_lines.scale.y = 0.0; marker_lines.scale.z = 0.0;
			marker_lines.points.reserve(4);
			// message with ZMP trajectory: lines
			marker_zmp = marker;
			marker_zmp.id = 2;
			marker_zmp.type = visualization_msgs::Marker::LINE_STRIP;
			marker_zmp.scale.x = point_size_param/2; marker_zmp.scale.y = 0.0; marker_zmp.scale.z = 0.0;
			marker_zmp.points.reserve(balance_history_length);
			marker_zmp.colors.reserve(balance_history_length);
			// message with ZMP trajectory: points
			marker_zmp_pt = marker;
			marker_zmp_pt.id = 3;
			marker_zmp_pt.type = visualization_msgs::Marker::POINTS;
			marker_zmp_pt.scale.x = point_size_param/2; marker_zmp_pt.scale.y = point_size_param/2; marker_zmp_pt.scale.z = 0.0;
			marker_zmp_pt.points.reserve(balance_history_length);
			marker_zmp_pt.colors.reserve(balance_history_length);
			// message with COM trajectory: lines
			marker_cog = marker;
			marker_cog.id = 4;
			marker_cog.type = visualization_msgs::Marker::LINE_STRIP;
			marker_cog.scale.x = point_size_param/2; marker_cog.scale.y = 0.0; marker_cog.scale.z = 0.0;
			marker_cog.points.reserve(balance_history_length);
			marker_cog.colors.reserve(balance_history_length);
			// message with COM trajectory: points
			marker_cog_pt = marker;
			marker_cog_pt.id = 5;
			marker_cog_pt.type = visualization_msgs::Marker::POINTS;
			marker_cog_pt.scale.x = point_size_param/2; marker_cog_pt.scale.y = point_size_param/2; marker_cog_pt.scale.z = 0.0;
			marker_cog_pt.points.reserve(balance_history_length);
			marker_cog_pt.colors.reserve(balance_history_length);
		}

		void visualizeBalance() {
			// get references to marker messages 
			Marker& marker_points = marker_balance.markers[0];
			Marker& marker_lines = marker_balance.markers[1];
			Marker& marker_zmp = marker_balance.markers[2];
			Marker& marker_zmp_pt = marker_balance.markers[3];
			Marker& marker_cog = marker_balance.markers[4];
			Marker& marker_cog_pt = marker_balance.markers[5];

			ros::Time stamp = ros::Time::now();
			
			// use balance message to display CoP, ZMP and CoM
			// points and colors arrays are already allocated
			// contact are assumed to be positioned in z = 0 plane
			marker_points.header.stamp = stamp;
			// CoP (z = 0)
			tf::pointKDLToMsg(balance.CoP, marker_points.points[0]);
			// ZMP (z = 0)
			tf::pointKDLToMsg(balance.ZMP, marker_points.points[1]);
			// CoM projection (z = 0)
			tf::pointKDLToMsg(balance.CoM, marker_points.points[2]);
			marker_points.points[2].z = 0.0;
			// CoM 
			tf::pointKDLToMsg(balance.CoM, marker_points.points[3]);

			// now add support polygone
			marker_lines.header.stamp = stamp;
			marker_lines.points.clear();
			for(int k = 0; k < balance.support_points.size(); k++) {
				geometry_msgs::Point point;
				tf::pointKDLToMsg(balance.support_points[k], point);
				marker_lines.points.push_back(point);
			}
			// close support polygon
			if (marker_lines.points.size() >= 2) {
				marker_lines.points.push_back(marker_lines.points.front());
			}

			// check if ZMP is inside support polygone
			bool in_balance = true;
			for(int k = 1; k < balance.support_points.size(); k++) {
				// calculate normalized normal vector to the side of support polygone
				KDL::Vector& p1 = balance.support_points[k-1];
				KDL::Vector& p2 = balance.support_points[k];
				KDL::Vector n(p1.y() - p2.y(), p2.x() - p1.x(), p1.x()*p2.y() - p2.x()*p1.y());
				double d = (p2-p1).Norm();
				n = (1.0/d) * n;
				// check if balance condition is fullfilled
				if (0.0 >= KDL::dot(n, KDL::Vector(balance.ZMP.x(), balance.ZMP.y(), 1.0))) {
					in_balance = false;
				}
			}
			ColorRGBAInit zmp_color = in_balance ? GREEN : RED;

			// add point to ZMP history
			marker_zmp.header.stamp = stamp;
			if (marker_zmp.points.size() < balance_history_length) {
				// add line point 
				marker_zmp.points.emplace_back();
				tf::pointKDLToMsg(balance.ZMP, marker_zmp.points.back());
				marker_zmp.colors.push_back(zmp_color);
			}
			else {
				// shift vector and add new point
				for(int k = 1; k < marker_zmp.points.size(); k++) {
					marker_zmp.points[k-1] = marker_zmp.points[k];
					marker_zmp.colors[k-1] = marker_zmp.colors[k];
					marker_zmp_pt.points[k-1] = marker_zmp_pt.points[k];
					marker_zmp_pt.colors[k-1] = marker_zmp_pt.colors[k];
				}
				tf::pointKDLToMsg(balance.ZMP, marker_zmp.points.back());
				marker_zmp.colors.back() = zmp_color;
			}
			// copy points
			marker_zmp_pt.points = marker_zmp.points;
			marker_zmp_pt.colors = marker_zmp.colors;

			// add point to CoM history
			marker_cog.header.stamp = stamp;
			if (marker_cog.points.size() < balance_history_length) {
				// add line point 
				marker_cog.points.emplace_back();
				tf::pointKDLToMsg(balance.CoM, marker_cog.points.back());
				marker_cog.points.back().z = 0.0;
				marker_cog.colors.push_back(LIGHT_BLUE);
			}
			else {
				// shift vector and add new point
				for(int k = 1; k < marker_cog.points.size(); k++) {
					marker_cog.points[k-1] = marker_cog.points[k];
					marker_cog.colors[k-1] = marker_cog.colors[k];
				}
				tf::pointKDLToMsg(balance.CoM, marker_cog.points.back());
				marker_cog.points.back().z = 0.0;
			}
			// copy points
			marker_cog_pt.points = marker_cog.points;
			marker_cog_pt.colors = marker_cog.colors;

			// publish resulting message
			markers_pub.publish(marker_balance);
		}
};


const ColorRGBAInit DynamicsVisualizer::RED = ColorRGBAInit(1 ,0, 0);
const ColorRGBAInit DynamicsVisualizer::GREEN = ColorRGBAInit(0, 1, 0);
const ColorRGBAInit DynamicsVisualizer::MAGENTA = ColorRGBAInit(1, 0, 1);
const ColorRGBAInit DynamicsVisualizer::BLUE = ColorRGBAInit(0, 0, 1);
const ColorRGBAInit DynamicsVisualizer::LIGHT_BLUE = ColorRGBAInit(0, 1, 1);
const ColorRGBAInit DynamicsVisualizer::YELLOW = ColorRGBAInit(1, 1, 0);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sweetie_dynamics_visualizer");
	ROS_INFO("SweetieBot Dynamics Visualizer main.");

	std::unique_ptr<DynamicsVisualizer> visualizer(new DynamicsVisualizer());

	ros::spin();

	return 0;
}

