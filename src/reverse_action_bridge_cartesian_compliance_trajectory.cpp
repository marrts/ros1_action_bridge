// Copyright 2019 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <reverse_action_bridge/reverse_action_bridge.hpp>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <cartesian_msgs/CartesianComplianceTrajectoryAction.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include <cartesian_msgs/action/cartesian_compliance_trajectory.hpp>

template<typename T1, typename T2>
static void copy_vector3(const T2 & pt2, T1 & pt1)
{
  pt1.x = pt2.x;
  pt1.y = pt2.y;
  pt1.z = pt2.z;
}

template<typename T1, typename T2>
static void copy_vector4(const T2 & pt2, T1 & pt1)
{
  pt1.w = pt2.w;
  pt1.x = pt2.x;
  pt1.y = pt2.y;
  pt1.z = pt2.z;
}

template<typename T1, typename T2>
static void copy_pose(const T2 & pt2, T1 & pt1)
{
  copy_vector3(pt2.position, pt1.position);
  copy_vector4(pt2.orientation, pt1.orientation);
}

template<typename T1, typename T2>
static void copy_twist(const T2 & pt2, T1 & pt1)
{
  copy_vector3(pt2.linear, pt1.linear);
  copy_vector3(pt2.angular, pt1.angular);
}

template<typename T1, typename T2>
static void copy_wrench(const T2 & pt2, T1 & pt1)
{
  copy_vector3(pt2.force, pt1.force);
  copy_vector3(pt2.torque, pt1.torque);
}

template<typename T1, typename T2>
static void copy_point(const T2 & pt2, T1 & pt1)
{
  copy_pose(pt2.pose, pt1.pose);
  copy_twist(pt2.twist, pt1.twist);
  copy_wrench(pt2.wrench, pt1.wrench);
}

template<typename T1, typename T2>
static void copy_tolerance(const T2 & tolerance2, T1 & tolerance1)
{
  copy_vector3(tolerance2.position_error, tolerance1.position_error);
  copy_vector3(tolerance2.orientation_error, tolerance1.orientation_error);
  copy_twist(tolerance2.twist_error, tolerance1.twist_error);
  copy_wrench(tolerance2.wrench_error, tolerance1.wrench_error);
}

template<typename T1, typename T2>
static void copy_header2_1(const T2 & head2, T1 & head1)
{
  head1.frame_id = head2.frame_id;
  head1.stamp.sec = head2.stamp.sec;
  head1.stamp.nsec = head2.stamp.nanosec;
}

template<typename T1, typename T2>
static void copy_header1_2(const T1 & head1, T2 & head2)
{
  head2.frame_id = head1.frame_id;
  head2.stamp.sec = head1.stamp.sec;
  head2.stamp.nanosec = head1.stamp.nsec;
}

template<typename T1, typename T2>
static void copy_trajectory(const T2 & traj2, T1 & traj1)
{
//  copy_header2_1(traj1.header, traj2.header);

  size_t num = traj2.points.size();
  traj1.points.resize(num);
  for (size_t i = 0; i < num; ++i)
  {
    copy_point(traj2.points[i], traj1.points[i]);
  }

  copy_pose(traj2.tcp_offset, traj1.tcp_offset);
  traj1.tcp_frame = traj2.tcp_frame;
}

using CartesianTrajectoryActionBridge = ActionBridge<cartesian_msgs::CartesianComplianceTrajectoryAction,
    cartesian_msgs::action::CartesianComplianceTrajectory>;

template<>
void CartesianTrajectoryActionBridge::translate_goal_2_to_1(
  const ROS2Goal & goal2,
  ROS1Goal & goal1)
{
  copy_trajectory(goal2.trajectory, goal1.trajectory);
  copy_tolerance(goal2.path_tolerance, goal1.path_tolerance);
  copy_tolerance(goal2.goal_tolerance, goal1.goal_tolerance);
  copy_wrench(goal2.force, goal1.force);
  goal1.speed = goal2.speed;
}

template<>
void CartesianTrajectoryActionBridge::translate_result_1_to_2(
  ROS2Result & result2,
  const ROS1Result & result1)
{
  result2.success = result1.success;
  result2.err_msg = result1.err_msg;
}

template<>
void CartesianTrajectoryActionBridge::translate_feedback_1_to_2(
  ROS2Feedback & feedback2,
  const ROS1Feedback & feedback1)
{
  feedback2.tcp_frame = feedback1.tcp_frame;
//  copy_header1_2(feedback1.header, feedback2.header);
  copy_point(feedback1.desired, feedback2.desired);
  copy_point(feedback1.virtual_desired, feedback2.virtual_desired);
  copy_point(feedback1.actual, feedback2.actual);
  copy_point(feedback1.error, feedback2.error);
}

int main(int argc, char * argv[])
{
  return CartesianTrajectoryActionBridge::main("cartesian_compliance_trajectory", argc, argv);
}

