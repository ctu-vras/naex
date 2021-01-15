//
// Created by petrito1 on 10/1/20.
//

#ifndef NAEX_PLANNER_H
#define NAEX_PLANNER_H

#include <algorithm>
#include <boost/graph/graph_concepts.hpp>
#include <cmath>
#include <cstddef>
#include <Eigen/Dense>
#include <flann/flann.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mutex>
#include <naex/array.h>
#include <naex/buffer.h>
#include <naex/clouds.h>
#include <naex/exceptions.h>
#include <naex/iterators.h>
#include <naex/map.h>
#include <naex/nearest_neighbors.h>
#include <naex/params.h>
#include <naex/timer.h>
#include <naex/transforms.h>
#include <naex/types.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace naex
{
    typedef ValueIterator<Vertex> VertexIter;
    typedef ValueIterator<Edge> EdgeIter;
}  // namespace naex

#include <naex/graph.h>

namespace naex
{

    class Planner
    {
    public:
        Planner(ros::NodeHandle& nh, ros::NodeHandle& pnh):
                nh_(nh),
                pnh_(pnh),
                tf_(),
                position_name_("x"),
                normal_name_("normal_x"),
                map_frame_(""),
                robot_frame_("base_footprint"),
                robot_frames_(),
                max_cloud_age_(5.0),
                max_pitch_(static_cast<float>(30. / 180. * M_PI)),
                max_roll_(static_cast<float>(30. / 180. * M_PI)),
                empty_ratio_(2),
                filter_robots_(false),
                neighborhood_knn_(12),
                neighborhood_radius_(.5),
                min_normal_pts_(9),
                normal_radius_(0.5),
                max_nn_height_diff_(0.15),
                viewpoints_update_freq_(1.0),
                viewpoints_(),
                clearance_low_(0.15),
                clearance_high_(0.8),
                min_points_obstacle_(3),
                max_ground_diff_std_(0.1),
                max_ground_abs_diff_mean_(0.1),
                edge_min_centroid_offset_(0.75),
                min_dist_to_obstacle_(0.7),
                other_viewpoints_(),
                min_vp_distance_(1.5),
                max_vp_distance_(5.0),
                self_factor_(0.25),
                planning_freq_(0.5),
                initialized_(false),
                queue_size_(5),
//                params_(),
                map_()
        {
            Timer t;
            // Invalid position invokes exploration mode.
            last_request_.start.pose.position.x = std::numeric_limits<double>::quiet_NaN();
            last_request_.start.pose.position.y = std::numeric_limits<double>::quiet_NaN();
            last_request_.start.pose.position.z = std::numeric_limits<double>::quiet_NaN();
            last_request_.goal.pose.position.x = std::numeric_limits<double>::quiet_NaN();
            last_request_.goal.pose.position.y = std::numeric_limits<double>::quiet_NaN();
            last_request_.goal.pose.position.z = std::numeric_limits<double>::quiet_NaN();
            last_request_.tolerance = 32.;
            configure();
            ROS_INFO("Waiting for other robots...");
            // TODO: Avoid blocking here to be usable as nodelet.
            find_robots(map_frame_, ros::Time(), 15.f);
            Lock lock(initialized_mutex_);
            initialized_ = true;
            ROS_INFO("Initialized (%.3f s).", t.seconds_elapsed());
        }

        void update_params(const ros::WallTimerEvent& evt)
        {
            pnh_.param("max_nn_height_diff", max_nn_height_diff_, max_nn_height_diff_);
            pnh_.param("clearance_low", clearance_low_, clearance_low_);
            pnh_.param("clearance_high", clearance_high_, clearance_high_);
            pnh_.param("min_points_obstacle", min_points_obstacle_, min_points_obstacle_);
            pnh_.param("max_ground_diff_std", max_ground_diff_std_, max_ground_diff_std_);
            pnh_.param("max_ground_abs_diff_mean", max_ground_abs_diff_mean_, max_ground_abs_diff_mean_);
            pnh_.param("edge_min_centroid_offset", edge_min_centroid_offset_, edge_min_centroid_offset_);
            pnh_.param("min_dist_to_obstacle", min_dist_to_obstacle_, min_dist_to_obstacle_);
        }

        void configure()
        {
            pnh_.param("position_name", position_name_, position_name_);
            pnh_.param("normal_name", normal_name_, normal_name_);
            pnh_.param("map_frame", map_frame_, map_frame_);
            pnh_.param("robot_frame", robot_frame_, robot_frame_);
            pnh_.param("robot_frames", robot_frames_, robot_frames_);
            pnh_.param("max_cloud_age", max_cloud_age_, max_cloud_age_);
            pnh_.param("max_pitch", max_pitch_, max_pitch_);
            pnh_.param("max_roll", max_roll_, max_roll_);
            pnh_.param("neighborhood_knn", neighborhood_knn_, neighborhood_knn_);
            pnh_.param("neighborhood_radius", neighborhood_radius_, neighborhood_radius_);
            pnh_.param("min_normal_pts", min_normal_pts_, min_normal_pts_);
            pnh_.param("normal_radius", normal_radius_, normal_radius_);

            pnh_.param("max_nn_height_diff", max_nn_height_diff_, max_nn_height_diff_);
            pnh_.param("clearance_low", clearance_low_, clearance_low_);
            pnh_.param("clearance_high", clearance_high_, clearance_high_);
            pnh_.param("min_points_obstacle", min_points_obstacle_, min_points_obstacle_);
            pnh_.param("max_ground_diff_std", max_ground_diff_std_, max_ground_diff_std_);
            pnh_.param("max_ground_abs_diff_mean", max_ground_abs_diff_mean_, max_ground_abs_diff_mean_);
            pnh_.param("edge_min_centroid_offset", edge_min_centroid_offset_, edge_min_centroid_offset_);
            pnh_.param("min_dist_to_obstacle", min_dist_to_obstacle_, min_dist_to_obstacle_);

            pnh_.param("viewpoints_update_freq", viewpoints_update_freq_, viewpoints_update_freq_);
            pnh_.param("min_vp_distance", min_vp_distance_, min_vp_distance_);
            pnh_.param("max_vp_distance", max_vp_distance_, max_vp_distance_);
            pnh_.param("self_factor", self_factor_, self_factor_);
            pnh_.param("planning_freq", planning_freq_, planning_freq_);

            int num_input_clouds = 1;
            pnh_.param("num_input_clouds", num_input_clouds, num_input_clouds);
            pnh_.param("input_queue_size", queue_size_, queue_size_);
            pnh_.param("points_min_dist", map_.points_min_dist_, map_.points_min_dist_);
            pnh_.param("min_empty_cos", map_.min_empty_cos_, map_.min_empty_cos_);
            pnh_.param("empty_ratio", empty_ratio_, empty_ratio_);
            pnh_.param("filter_robots", filter_robots_, filter_robots_);

            bool among_robots = false;
            for (const auto& kv: robot_frames_)
            {
                if (kv.second == robot_frame_)
                {
                    among_robots = true;
                }
            }
            if (!among_robots)
            {
                ROS_INFO("Inserting robot frame among all robot frames.");
                robot_frames_["SELF"] = robot_frame_;
            }

            viewpoints_.reserve(size_t(7200. * viewpoints_update_freq_) * 3);
            other_viewpoints_.reserve(size_t(7200. * viewpoints_update_freq_) * 3 * robot_frames_.size());

            // C++14
//            tf_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(10.));
//            tf_sub_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
            tf_.reset(new tf2_ros::Buffer(ros::Duration(30.)));
            tf_sub_.reset(new tf2_ros::TransformListener(*tf_));

            normal_label_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("normal_label_cloud", 5);
            final_label_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("final_label_cloud", 5);
            path_cost_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("path_cost_cloud", 5);
            utility_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("utility_cloud", 5);
            final_cost_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("final_cost_cloud", 5);

            path_pub_ = nh_.advertise<nav_msgs::Path>("path", 5);
            minpos_path_pub_ = nh_.advertise<nav_msgs::Path>("minpos_path", 5);

            viewpoints_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("viewpoints", 5);
            other_viewpoints_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("other_viewpoints", 5);
            map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 5);
            map_dirty_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_dirty", 5);
            map_diff_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_diff", 5);

            cloud_sub_ = nh_.subscribe("input_map", queue_size_, &Planner::cloud_received, this);
            for (int i = 0; i < num_input_clouds; ++i)
            {
                std::stringstream ss;
                ss << "input_cloud_" << i;
                auto sub = nh_.subscribe(ss.str(), queue_size_, &Planner::input_cloud_received, this);
                input_cloud_subs_.push_back(sub);
            }

            viewpoints_update_timer_ =  nh_.createTimer(ros::Rate(viewpoints_update_freq_),
                    &Planner::gather_viewpoints, this);
            planning_timer_ =  nh_.createTimer(ros::Rate(planning_freq_),
                    &Planner::planning_timer_cb, this);
            update_params_timer_ = nh_.createWallTimer(ros::WallDuration(2.0),
                    &Planner::update_params, this);

            get_plan_service_ = nh_.advertiseService("get_plan", &Planner::plan, this);
        }

        void gather_viewpoints(const ros::TimerEvent& event)
        {
            ROS_DEBUG("Gathering viewpoints.");
            if (map_frame_.empty())
            {
                ROS_ERROR("Could not gather robot positions due to missing map frame.");
                return;
            }
            Lock lock(viewpoints_mutex_);
            for (const auto& kv: robot_frames_)
            {
                const auto& frame = kv.second;
                try
                {
                    // Get last transform available (don't wait).
                    auto tf = tf_->lookupTransform(map_frame_, frame, ros::Time());
                    if (frame == robot_frame_)
                    {
                        viewpoints_.push_back(tf.transform.translation.x);
                        viewpoints_.push_back(tf.transform.translation.y);
                        viewpoints_.push_back(tf.transform.translation.z);
                    }
                    else
                    {
                        other_viewpoints_.push_back(tf.transform.translation.x);
                        other_viewpoints_.push_back(tf.transform.translation.y);
                        other_viewpoints_.push_back(tf.transform.translation.z);
                    }
                }
                catch (const tf2::TransformException& ex)
                {
                    ROS_WARN("Could not get robot %s position: %s.", frame.c_str(), ex.what());
                    continue;
                }
            }
            auto now = ros::Time::now();
            sensor_msgs::PointCloud2 vp_cloud;
            flann::Matrix<Elem> vp(viewpoints_.data(), viewpoints_.size(), 3);
            create_xyz_cloud(vp, vp_cloud);
            vp_cloud.header.frame_id = map_frame_;
            vp_cloud.header.stamp = now;
            viewpoints_pub_.publish(vp_cloud);

            sensor_msgs::PointCloud2 other_vp_cloud;
            flann::Matrix<Elem> other_vp(other_viewpoints_.data(), other_viewpoints_.size(), 3);
            create_xyz_cloud(other_vp, other_vp_cloud);
            other_vp_cloud.header.frame_id = map_frame_;
            other_vp_cloud.header.stamp = now;
            other_viewpoints_pub_.publish(other_vp_cloud);
        }

        void trace_path_indices(Vertex start, Vertex goal, const Buffer<Vertex>& predecessor,
                std::vector<Vertex>& path_indices)
        {
            assert(predecessor[start] == start);
            Vertex v = goal;
            while (v != start)
            {
                path_indices.push_back(v);
                v = predecessor[v];
            }
            path_indices.push_back(v);
            std::reverse(path_indices.begin(), path_indices.end());
        }

        void append_path(const std::vector<Vertex>& path_indices,
//                const flann::Matrix<Elem>& points,
//                const flann::Matrix<Elem>& normals,
                const std::vector<Point>& points,
                nav_msgs::Path& path)
        {
            if (path_indices.empty())
            {
                return;
            }
            path.poses.reserve(path.poses.size() + path_indices.size());
            for (const auto& v: path_indices)
            {
                geometry_msgs::PoseStamped pose;
//                pose.pose.position.x = points[v][0];
//                pose.pose.position.y = points[v][1];
//                pose.pose.position.z = points[v][2];
                pose.pose.position.x = points[v].position_[0];
                pose.pose.position.y = points[v].position_[1];
                pose.pose.position.z = points[v].position_[2];
                pose.pose.orientation.w = 1.;
                if (!path.poses.empty())
                {
                    Vec3 x(pose.pose.position.x - path.poses.back().pose.position.x,
                            pose.pose.position.y - path.poses.back().pose.position.y,
                            pose.pose.position.z - path.poses.back().pose.position.z);
                    x.normalize();
//                    Vec3Map z(normals[v]);
                    Vec3 z = ConstVec3Map(points[v].normal_);
                    // Fix z direction to be consistent with the previous pose.
                    // As we start from the current robot pose with correct z
                    // orientation, all following z directions get corrected.
//                    Quat q_prev(path.poses.back().pose.orientation.w,
//                            path.poses.back().pose.orientation.x,
//                            path.poses.back().pose.orientation.y,
//                            path.poses.back().pose.orientation.z);
//                    Mat3 R_prev;
//                    R_prev = q_prev;
//                    Vec3 z_prev = R_prev.col(2);
//                    if (z.dot(z_prev) < 0.)
//                    {
//                        z = -z;
//                    }
                    // Assume map z points upward.
                    if (z.dot(Vec3(0.f, 0.f, 1.f)) < 0.)
                    {
                        z = -z;
                    }

                    Mat3 m;
//                    m.row(0) = x;
//                    m.row(1) = z.cross(x);
//                    m.row(2) = z;
                    m.col(0) = x;
                    m.col(1) = z.cross(x);
                    m.col(2) = z;
                    Quat q;
                    q = m;
                    pose.pose.orientation.x = q.x();
                    pose.pose.orientation.y = q.y();
                    pose.pose.orientation.z = q.z();
                    pose.pose.orientation.w = q.w();
                }
                path.poses.push_back(pose);
            }
        }

        Buffer<Elem> viewpoint_dist(const flann::Matrix<Elem>& points)
        {
            Timer t;
            Buffer<Elem> dist(points.rows);
            std::vector<Elem> vp_copy;
            {
                Lock lock(viewpoints_mutex_);
                if (viewpoints_.empty())
                {
                    ROS_WARN("No viewpoints gathered. Return infinity.");
                    std::fill(dist.begin(), dist.end(), std::numeric_limits<Elem>::infinity());
                    return dist;
                }
                vp_copy = viewpoints_;
            }
            size_t n_vp = vp_copy.size() / 3;
            ROS_INFO("Number of viewpoints: %lu.", n_vp);
            flann::Matrix<Elem> vp(vp_copy.data(), n_vp, 3);
//            flann::Index<flann::L2_3D<Elem>> vp_index(vp, flann::KDTreeIndexParams(2));
            flann::Index<flann::L2_3D<Elem>> vp_index(vp, flann::KDTreeSingleIndexParams());
            vp_index.buildIndex();
            Query<Elem> vp_query(vp_index, points, 1);
            return vp_query.dist_buf_;
        }

        Buffer<Elem> other_viewpoint_dist(const flann::Matrix<Elem>& points)
        {
            Timer t;
            Buffer<Elem> dist(points.rows);
            std::vector<Elem> vp_copy;
            {
                Lock lock(viewpoints_mutex_);
                if (other_viewpoints_.empty())
                {
                    ROS_WARN("No viewpoints gathered from other robots. Return infinity.");
                    std::fill(dist.begin(), dist.end(), std::numeric_limits<Elem>::infinity());
                    return dist;
                }
                vp_copy = other_viewpoints_;
            }
            size_t n_vp = vp_copy.size() / 3;
            ROS_INFO("Number of viewpoints from other robots: %lu.", n_vp);
            flann::Matrix<Elem> vp(vp_copy.data(), n_vp, 3);
//            flann::Index<flann::L2_3D<Elem>> vp_index(vp, flann::KDTreeIndexParams(2));
            flann::Index<flann::L2_3D<Elem>> vp_index(vp, flann::KDTreeSingleIndexParams());
            vp_index.buildIndex();
            Query<Elem> vp_query(vp_index, points, 1);
            return vp_query.dist_buf_;
        }

        void input_map_received(const sensor_msgs::PointCloud2& cloud)
        {

            Timer t;
            const size_t n_pts = cloud.height * cloud.width;
            sensor_msgs::PointCloud2ConstIterator<Elem> it_points(cloud, "x");
//            const flann::Matrix<Elem> points(const_cast<Elem*>(&it_points[0]), n_pts, 3, cloud.point_step);
            sensor_msgs::PointCloud2ConstIterator<Elem> it_normals(cloud, "normal_x");
//            const flann::Matrix<Elem> normals(const_cast<Elem*>(&it_normals[0]), n_pts, 3, cloud.point_step);
            Buffer<Elem> points_buf(3 * n_pts);
            Buffer<Elem> normals_buf(3 * n_pts);
            auto it_points_dst = points_buf.begin();
            auto it_normals_dst = normals_buf.begin();
            for (size_t i = 0; i < n_pts; ++i, ++it_points, ++it_normals)
            {
                *it_points_dst++ = it_points[0];
                *it_points_dst++ = it_points[1];
                *it_points_dst++ = it_points[2];
                *it_normals_dst++ = it_normals[0];
                *it_normals_dst++ = it_normals[1];
                *it_normals_dst++ = it_normals[2];
            }
            flann::Matrix<Elem> points(points_buf.begin(), n_pts, 3);
            flann::Matrix<Elem> normals(normals_buf.begin(), n_pts, 3);
            ROS_INFO("Copy of %lu points and normals: %.3f s.", n_pts, t.seconds_elapsed());
        }

//        bool plan(const geometry_msgs::PoseStamped& start)
        bool plan(nav_msgs::GetPlanRequest& req, nav_msgs::GetPlanResponse& res)
        {
            ROS_WARN("Don't plan for now.");
            return false;
            Timer t;
            {
                Lock lock(initialized_mutex_);
                if (!initialized_)
                {
                    ROS_WARN("Won't plan. Waiting for other robots...");
                    return false;
                }
            }
            ROS_INFO("Planning from [%.1f, %.1f, %.1f] to [%.1f, %.1f, %.1f] with tolerance %.1f.",
                    req.start.pose.position.x, req.start.pose.position.y, req.start.pose.position.z,
                    req.goal.pose.position.x, req.goal.pose.position.y, req.goal.pose.position.z,
                    req.tolerance);
            {
                Lock lock(last_request_mutex_);
                last_request_ = req;
            }
            geometry_msgs::PoseStamped start = req.start;
            if (std::isnan(start.pose.position.x)
                    || std::isnan(start.pose.position.y)
                    || std::isnan(start.pose.position.z))
            {
                try
                {
                    auto tf = tf_->lookupTransform(map_frame_, robot_frame_,
                            ros::Time::now(), ros::Duration(5.));
                    transform_to_pose(tf, start);
                }
                catch (const tf2::TransformException& ex)
                {
                    ROS_ERROR("Could not get robot %s position in map %s: %s.",
                            robot_frame_.c_str(), map_frame_.c_str(), ex.what());
                    return false;
                }
            }
            const geometry_msgs::PoseStamped goal = req.goal;

//            FlannMat points;
//            FlannMat normals;
//            FlannMat viewpoints;
//            FlannIndexPtr index;
//            Buffer<uint16_t> occupied;
//            Buffer<uint16_t> empty;
//            map_.snapshot(points, normals, viewpoints, index, occupied, empty);

            size_t min_map_points = 64;
//            if (points.rows < min_map_points)
//            if (map_.get_cloud().size() < min_map_points)
            if (map_.size() < min_map_points)
            {
                ROS_ERROR("Cannot plan in map with %lu < %lu points.",
                    map_.size(), min_map_points);
                return false;
            }

            // Moved to input_map_received above.

            // Get robot positions.
            std::vector<Elem> robots;
            if (filter_robots_)
            {
                // Don't wait for robot positions for planning.
                robots = find_robots(map_frame_, ros::Time(), 0.f);
            }

            // Compute preliminary point labels based on normals.
//            Graph g(points, normals, occupied, empty, max_pitch_, max_roll_, uint16_t(empty_ratio_));
//            g.k_ = neighborhood_knn_;
//            g.radius_ = neighborhood_radius_;

            // Update NN graph and recompute affected labels.
            // Clear dirty state.
            // TODO: Deal with occupancy on merging.
//            g.compute_occupancy_labels(robots);
            // Construct NN graph.
            // TODO: Index rebuild incrementally with new points.
//            g.build_index();
//            auto dirty = map_.collect_points_to_update();
//            map_.update_neighborhood(dirty.begin(), dirty.end());
//            map_.compute_features(dirty.begin(), dirty.end());
//            map_.compute_normal_labels(dirty.begin(), dirty.end());
//            map_.compute_labels(dirty.begin(), dirty.end());

            // Create debug cloud for visualization of intermediate results.
            sensor_msgs::PointCloud2 debug_cloud;
//            create_debug_cloud(points, normals, debug_cloud);
            map_.create_debug_cloud(debug_cloud);

            debug_cloud.header.frame_id = map_frame_;
            // TODO: Refresh on sending?
            debug_cloud.header.stamp = ros::Time::now();
            // Reconstruct original 2D shape.
            debug_cloud.height = 1;
//            debug_cloud.width = uint32_t(map_.get_cloud().size());
            debug_cloud.width = uint32_t(map_.size());
//            g.compute_graph(neighborhood_knn_, neighborhood_radius_);
//            g.recompute_normals(min_normal_pts_, normal_radius_);
//            g.compute_graph_features(min_normal_pts_, normal_radius_, edge_min_centroid_offset_);
//            fill_field("num_normal_pts", g.num_normal_pts_.begin(), debug_cloud);
//            fill_field("ground_diff_std", g.ground_diff_std_.begin(), debug_cloud);
//            g.compute_normal_labels();
//            fill_field("normal_label", g.labels_.begin(), debug_cloud);
//            normal_label_cloud_pub_.publish(debug_cloud);
            // Adjust points labels using constructed NN graph.
//            g.compute_final_labels(max_nn_height_diff_);
//            g.compute_final_labels(max_nn_height_diff_, clearance_low_, clearance_high_, min_points_obstacle_,
//                    max_ground_diff_std_, max_ground_abs_diff_mean_, min_dist_to_obstacle_);
//            fill_field("ground_diff_min", g.ground_diff_min_.begin(), debug_cloud);
//            fill_field("ground_diff_max", g.ground_diff_max_.begin(), debug_cloud);
//            fill_field("ground_abs_diff_mean", g.ground_abs_diff_mean_.begin(), debug_cloud);
//            fill_field("num_obstacle_pts", g.num_obstacle_pts_.begin(), debug_cloud);
//            fill_field("final_label", g.labels_.begin(), debug_cloud);
//            final_label_cloud_pub_.publish(debug_cloud);

            // Use the nearest traversable point to robot as the starting point.
            Vec3 start_position(float(start.pose.position.x),
                                float(start.pose.position.y),
                                float(start.pose.position.z));
            size_t tol = 32;
            if (req.tolerance > 0.)
            {
//                tol = std::min(size_t(req.tolerance), g.points_.rows);
//                tol = std::min(size_t(req.tolerance), map_.get_cloud().size());
                tol = std::min(size_t(req.tolerance), map_.cloud_.size());
            }
//            Query<Elem> start_query(g.points_index_, flann::Matrix<Elem>(start_position.data(), 1, 3), tol);
            std::vector<Elem> traversable;
            traversable.reserve(tol);
{
            // TODO: Make tolerance a distance instead of number of nearest points.
            Lock lock(map_.index_mutex_);
            Query<Value> start_query(*map_.index_, FlannMat(start_position.data(), 1, 3), tol);
            // Get traversable points.
            for (const auto& v: start_query.nn_buf_)
            {
//                if (g.labels_[v] == TRAVERSABLE || g.labels_[v] == EDGE)
//                if (map_.get_cloud()[v].functional_label_ == TRAVERSABLE
//                        || map_.get_cloud()[v].functional_label_ == EDGE)
                if (map_.cloud_[v].flags_ & TRAVERSABLE
                        || map_.cloud_[v].flags_ & EDGE)
                {
                    traversable.push_back(v);
                }
            }
            if (traversable.empty())
            {
                ROS_ERROR("No traversable point near start [%.1f, %.1f, %.1f].",
                        start_position.x(), start_position.y(), start_position.z());
                return false;
            }
}
//            Vertex v_start = start_query.nn_buf_[0];
            // Get random traversable point as start.
            Vertex v_start = traversable[std::rand() % traversable.size()];
//            Elem min_dist = std::numeric_limits<Elem>::infinity();
//            for (const auto& v: start_query.nn_buf_)
//            {
//                if (g.labels_[v] != TRAVERSABLE)
//                    continue;
//                if (g.labels_[v] )
//                Elem dist = (ConstVec3Map(points[v]) - start_position).norm();
//
//                if (dist < min_dist)
//                {
//                    v_start = v;
//                    min_dist = dist;
//                }
//            }
//            ROS_INFO("Planning from [%.1f, %.1f, %.1f], robot at [%.1f, %.1f, %.1f], %lu traversable points nearby.",
//                    g.points_[v_start][0], g.points_[v_start][1], g.points_[v_start][2],
//                    start_position.x(), start_position.y(), start_position.z(), traversable.size());
            ROS_INFO("Request from [%.1f, %.1f, %.1f],"
                     "robot at [%.1f, %.1f, %.1f] with %lu traversable points nearby,"
                     "planning from [%.1f, %.1f, %.1f].",
                     req.start.pose.position.x,
                     req.start.pose.position.y,
                     req.start.pose.position.z,
                     map_.cloud_[v_start].position_[0],
                     map_.cloud_[v_start].position_[1],
                     map_.cloud_[v_start].position_[2],
                     traversable.size(),
                     start_position.x(),
                     start_position.y(),
                     start_position.z());

            // TODO: Append starting pose as a special vertex with orientation dependent edges.
            // Note, that for some worlds and robots, the neighborhood must be quite large to get traversable points.
            // See e.g. X1 @ cave_circuit_practice_01.
            Graph g(map_);
            // Plan in NN graph with approx. travel time costs.
            Buffer<Vertex> predecessor(g.num_vertices());
            Buffer<Value> path_costs(g.num_vertices());
            EdgeCosts edge_costs(map_);
            boost::typed_identity_property_map<Vertex> index_map;

            Timer t_dijkstra;
            // TODO: Stop via exception if needed.
            // boost::dijkstra_shortest_paths(g, ::Graph::V(0),
            boost::dijkstra_shortest_paths_no_color_map(g, v_start,
//                    &predecessor[0], &path_costs[0], edge_costs,
                    predecessor.begin(), path_costs.begin(), edge_costs,
                    index_map,
                    std::less<Elem>(), boost::closed_plus<Elem>(), std::numeric_limits<Elem>::infinity(), Elem(0.),
                    boost::dijkstra_visitor<boost::null_visitor>());
            ROS_DEBUG("Dijkstra (%u pts): %.3f s.", g.num_vertices(), t_dijkstra.seconds_elapsed());
            fill_field("path_cost", path_costs.begin(), debug_cloud);
            path_cost_cloud_pub_.publish(debug_cloud);

            // If planning for given goal, get the closest feasible path.
            if (std::isfinite(goal.pose.position.x)
                    && std::isfinite(goal.pose.position.y)
                    && std::isfinite(goal.pose.position.z))
            {
                Vec3 goal_position(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
                Vertex v_goal = INVALID_VERTEX;
                Elem best_dist = std::numeric_limits<Elem>::infinity();
                for (Index v = 0; v < path_costs.size(); ++v)
                {
                    if (std::isinf(path_costs[v]))
                    {
                        continue;
                    }
//                    Elem dist = (ConstVec3Map(points[v]) - goal_position).norm();
                    Value dist = (ConstVec3Map(map_.cloud_[v].position_) - goal_position).norm();
                    if (dist < best_dist)
                    {
                        v_goal = v;
                        best_dist = dist;
                    }
                }
//                Vec3 goal_position(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
//                size_t tol = std::min(size_t(req.tolerance), g.points_.rows);
//                Query<Elem> goal_query(g.points_index_, flann::Matrix<Elem>(goal_position.data(), 1, 3), tol);
//                // Get traversable points.
//                std::vector<Elem> traversable;
//                traversable.reserve(tol);
//                for (const auto& v: goal_query.nn_buf_)
//                {
//                    if (g.labels_[v] == TRAVERSABLE || g.labels_[v] == EDGE)
//                    {
//                        traversable.push_back(v);
//                    }
//                }
//                if (traversable.empty())
//                {
//                    ROS_ERROR("No traversable point near goal [%.1f, %.1f, %.1f].",
//                            goal_position.x(), goal_position.y(), goal_position.z());
//                    return false;
//                }
//                // Get random traversable point as goal.
//                v_goal = traversable[std::rand() % traversable.size()];
//
//                ROS_INFO("Planning to [%.1f, %.1f, %.1f], %lu traversable points nearby.",
//                        g.points_[v_goal][0], g.points_[v_goal][1], g.points_[v_goal][2],
//                        goal_position.x(), goal_position.y(), goal_position.z(), traversable.size());

                if (v_goal == INVALID_VERTEX)
                {
                    ROS_WARN("No feasible path to [%.2f, %.2f, %.2f] found (%.3f s).",
                            goal_position.x(), goal_position.y(), goal_position.z(), t.seconds_elapsed());
                    return false;
                }
                std::vector<Vertex> path_indices;
                trace_path_indices(v_start, v_goal, predecessor, path_indices);
                res.plan.header.frame_id = map_frame_;
                res.plan.header.stamp = ros::Time::now();
                res.plan.poses.push_back(start);
//                append_path(path_indices, points, normals, res.plan);
                append_path(path_indices, map_.cloud_, res.plan);
                ROS_DEBUG("Path of length %lu (%.3f s).", res.plan.poses.size(), t.seconds_elapsed());
                return true;
            }

            // Compute vertex utility as minimum observation distance.
//            Buffer<Elem> vp_dist = viewpoint_dist(points);
//            Buffer<Elem> other_vp_dist = other_viewpoint_dist(points);
//            assert(vp_dist.size() == points.rows);
//            assert(other_vp_dist.size() == points.rows);
            Buffer<Elem> vp_dist = viewpoint_dist(map_.position_matrix());
            Buffer<Elem> other_vp_dist = other_viewpoint_dist(map_.position_matrix());
//            assert(vp_dist.size() == points.rows);
//            assert(other_vp_dist.size() == points.rows);
            Buffer<Elem> utility(vp_dist.size());

            auto it_vp = vp_dist.begin();
            auto it_other_vp = other_vp_dist.begin();
            auto it_utility = utility.begin();
            for (size_t i = 0; i < utility.size(); ++i, ++it_vp, ++it_other_vp, ++it_utility)
            {
                // Multiply the (clipped) Euclidean distance to encourage exploration.
//                *it = 3.f * std::min(std::max(std::sqrt(*it) - 2.f * neighborhood_radius_, 0.f), 5.f);
//                *it = std::min(std::max(std::sqrt(*it) - min_vp_distance_, 0.f), max_vp_distance_) / max_vp_distance_;
                const auto vp_dist = std::sqrt(*it_vp);
                const auto vp_dist_all = std::min(vp_dist, std::sqrt(*it_other_vp));
                const auto util = std::min(std::max(vp_dist - min_vp_distance_, 0.f), max_vp_distance_) / max_vp_distance_;
                const auto util_all = std::min(std::max(vp_dist_all - min_vp_distance_, 0.f), max_vp_distance_) / max_vp_distance_;
                *it_utility = std::max(self_factor_ * util, util_all);
                // Multiply with edge neighbors.
//                *it_utility *= (1 + g.num_edge_neighbors_[i]);
                *it_utility *= (1 + map_.cloud_[i].num_edge_neighbors_);
                // Prefer frontiers in a specific subspace (e.g. positive x).
                // TODO: Ensure frame subt is used here.
//                *it_utility /= std::max(-points[i][0] + 9.f, 1.f);
//                if (points[i][0] >= -60. && points[i][0] <= 0.
//                        && points[i][1] >= -30. && points[i][1] <= 30.
//                        && points[i][2] >= -30. && points[i][0] <= 30.)
//                {
//                    Elem dist_from_origin = ConstVec3Map(points[i]).norm();
//                    *it_utility /= (1. + std::pow(dist_from_origin, 4.f));
//                }

                if (map_.cloud_[i].position_[0] >= -60. && map_.cloud_[i].position_[0] <= 0.
                    && map_.cloud_[i].position_[1] >= -30. && map_.cloud_[i].position_[1] <= 30.
                    && map_.cloud_[i].position_[2] >= -30. && map_.cloud_[i].position_[0] <= 30.)
                {
                    Elem dist_from_origin = ConstVec3Map(map_.cloud_[i].position_).norm();
                    *it_utility /= (1. + std::pow(dist_from_origin, 4.f));
                }
            }
            fill_field("utility", utility.begin(), debug_cloud);
            utility_cloud_pub_.publish(debug_cloud);

            // Publish final cost cloud.
            Buffer<Elem> final_costs(path_costs.size());
            Elem goal_cost = std::numeric_limits<Elem>::infinity();
            Vertex v_goal = INVALID_VERTEX;
            Vertex v = 0;
            auto it_path_cost = path_costs.begin();
            it_utility = utility.begin();
            auto it_final_cost = final_costs.begin();
            for (; it_path_cost != path_costs.end(); ++v, ++it_path_cost, ++it_utility, ++it_final_cost)
            {
                // *it_final_cost = *it_path_cost - *it_utility;
//                *it_final_cost = std::log(*it_path_cost / (*it_utility + 1e-6f));
                *it_final_cost = *it_path_cost / (*it_utility + 1e-6f);

                // Avoid short paths and paths with no utility.
                if (*it_final_cost < goal_cost && *it_path_cost > neighborhood_radius_ && *it_utility > 0.)
                {
                    goal_cost = *it_final_cost;
                    v_goal = v;
                }
            }
            if (v_goal == INVALID_VERTEX)
            {
                ROS_ERROR("No valid path/goal found.");
                return false;
            }

//            ROS_INFO("Goal position: %.1f, %.1f, %.1f m: cost %.3g, utility %.3g, final cost %.3g.",
//                    points[v_goal][0], points[v_goal][1], points[v_goal][2],
//                    path_costs[v_goal], utility[v_goal], final_costs[v_goal]);
            ROS_INFO("Goal position: %.1f, %.1f, %.1f m: cost %.3g, utility %.3g, final cost %.3g.",
                     map_.position(v_goal)[0], map_.position(v_goal)[1], map_.position(v_goal)[2],
                     path_costs[v_goal], utility[v_goal], final_costs[v_goal]);
            fill_field("final_cost", final_costs.begin(), debug_cloud);
            final_cost_cloud_pub_.publish(debug_cloud);

            // TODO: Remove inf from path cost for visualization?

            std::vector<Vertex> path_indices;
            trace_path_indices(v_start, v_goal, predecessor, path_indices);

//            nav_msgs::Path path;
            res.plan.header.frame_id = map_frame_;
            res.plan.header.stamp = ros::Time::now();
            res.plan.poses.push_back(start);
//            append_path(path_indices, points, normals, res.plan);
            append_path(path_indices, map_.cloud_, res.plan);
            ROS_DEBUG("Path of length %lu (%.3f s).", res.plan.poses.size(), t.seconds_elapsed());
            return true;
        }

        void cloud_received(const sensor_msgs::PointCloud2::ConstPtr& cloud)
        {
            ROS_INFO("Cloud received (%u points).", cloud->height * cloud->width);

            // TODO: Build map from all aligned input clouds (interp tf).
            // TODO: Recompute normals.
            if (cloud->row_step != cloud->point_step * cloud->width)
            {
                ROS_ERROR("Skipping cloud with unsupported row step.");
                return;
            }
            const auto age = (ros::Time::now() - cloud->header.stamp).toSec();
            if (age > max_cloud_age_)
            {
                ROS_INFO("Skipping cloud %.1f s > %.1f s old.", age, max_cloud_age_);
                return;
            }
            if (!map_frame_.empty() && map_frame_ != cloud->header.frame_id)
            {
                ROS_ERROR("Cloud frame %s does not match specified map frame %s.",
                        cloud->header.frame_id.c_str(), map_frame_.c_str());
                return;
            }

            // TODO: Allow x[3] or x,y,z and normal[3] or normal_x,y,z.
            const auto field_x = find_field(*cloud, position_name_);
            if (!field_x)
            {
                ROS_ERROR("Skipping cloud without positions.");
                return;
            }
            if (field_x->datatype != sensor_msgs::PointField::FLOAT32)
            {
                ROS_ERROR("Skipping cloud with unsupported type %u.", field_x->datatype);
                return;
            }

            const auto field_nx = find_field(*cloud, normal_name_);
            if (!field_nx)
            {
                ROS_ERROR("Skipping cloud without normals.");
                return;
            }
            if (field_nx->datatype != sensor_msgs::PointField::FLOAT32)
            {
                ROS_ERROR("Skipping cloud with unsupported normal type %u.", field_nx->datatype);
                return;
            }

            geometry_msgs::PoseStamped start;
            try
            {
                auto tf = tf_->lookupTransform(cloud->header.frame_id, robot_frame_, ros::Time::now(),
                        ros::Duration(5.));
                transform_to_pose(tf, start);
            }
            catch (const tf2::TransformException& ex)
            {
                ROS_ERROR("Could not get robot position: %s.", ex.what());
                return;
            }
            // TODO: Update whole map with the input map cloud.
//            plan(*cloud, start);
        }

        void planning_timer_cb(const ros::TimerEvent& event)
        {
            ROS_DEBUG("Planning timer callback.");
            Timer t;
            nav_msgs::GetPlanRequest req;
            {
                Lock lock(last_request_mutex_);
                req = last_request_;
            }
            nav_msgs::GetPlanResponse res;
            if (!plan(req, res))
            {
                return;
            }
            path_pub_.publish(res.plan);
            ROS_INFO("Planning robot %s path (%lu poses) in map %s: %.3f s.",
                    robot_frame_.c_str(), res.plan.poses.size(), map_frame_.c_str(), t.seconds_elapsed());
        }

        std::vector<Value> find_robots(const std::string& frame, const ros::Time& stamp, float timeout)
        {
            Timer t;
            std::vector<Value> robots;
            robots.reserve(3 * robot_frames_.size());
            for (const auto kv: robot_frames_)
            {
                if (robot_frame_ == kv.second)
                {
                    continue;
                }
                ros::Duration timeout_duration(std::max(timeout - (ros::Time::now() - stamp).toSec(), 0.));
                geometry_msgs::TransformStamped tf;
                try
                {
                    tf = tf_->lookupTransform(frame, stamp, kv.second, stamp, map_frame_, timeout_duration);
                }
                catch (const tf2::TransformException& ex)
                {
                    ROS_WARN("Could not get %s pose in %s: %s.",
                            kv.second.c_str(), frame.c_str(), ex.what());
                    continue;
                }
                robots.push_back(static_cast<Value>(tf.transform.translation.x));
                robots.push_back(static_cast<Value>(tf.transform.translation.y));
                robots.push_back(static_cast<Value>(tf.transform.translation.z));
                ROS_INFO("Robot %s found in %s at [%.1f, %.1f, %.1f].", kv.second.c_str(), frame.c_str(),
                        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
            }
            ROS_INFO("%lu / %lu robots found in %.3f s (timeout %.3f s).",
                     robots.size() / 3, robot_frames_.size(), t.seconds_elapsed(), timeout);
            return robots;
        }

        void check_initialized()
        {
            Lock lock(initialized_mutex_);
            if (!initialized_)
            {
                throw NotInitialized("Not initialized. Waiting for other robots.");
            }
        }

        void filter_out_robots(
            const sensor_msgs::PointCloud2::ConstPtr& cloud,
            std::vector<uint8_t> &keep)
        {

        }

        void input_cloud_received(const sensor_msgs::PointCloud2::ConstPtr& cloud)
        {
//            {
//                Lock lock(initialized_mutex_);
//                if (!initialized_)
//                {
//                    ROS_WARN("Discarding input cloud. Waiting for other robots...");
//                    return;
//                }
//            }
            check_initialized();
            const Index n_pts = cloud->height * cloud->width;
            ROS_DEBUG("Input cloud from %s with %u points received.",
                      cloud->header.frame_id.c_str(), n_pts);
            Timer t;
            double timeout_ = 5.;
            ros::Duration timeout(std::max(timeout_ - (ros::Time::now() - cloud->header.stamp).toSec(), 0.));
            geometry_msgs::TransformStamped tf;
            try
            {
                tf = tf_->lookupTransform(map_frame_, cloud->header.frame_id, cloud->header.stamp, timeout);
            }
            catch (const tf2::TransformException &ex)
            {
                ROS_ERROR("Could not transform input cloud from %s into map %s: %s.",
                        cloud->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
                return;
            }
            ROS_DEBUG("Had to wait %.3f s for input cloud transform.", t.seconds_elapsed());

            Quat rotation(tf.transform.rotation.w,
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z);
            Eigen::Translation3f translation(tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z);
            Eigen::Affine3f transform = translation * rotation;

            // TODO: Update map occupancy based on reconstructed surface of 2D cloud.

//            Buffer<Elem> robots;
            std::vector<Elem> robots;
            if (filter_robots_)
            {
                Timer t;
                robots = find_robots(cloud->header.frame_id, cloud->header.stamp, 3.f);
            }

            Vec3 origin = transform * Vec3(0., 0., 0.);
            flann::Matrix<Elem> origin_mat(origin.data(), 1, 3);

            sensor_msgs::PointCloud2ConstIterator<Elem> it_points(*cloud, position_name_);
            Buffer<Elem> points_buf(3 * n_pts);
            auto it_points_dst = points_buf.begin();
            Index n_added = 0;
            for (Index i = 0; i < n_pts; ++i, ++it_points)
            {
                ConstVec3Map src(&it_points[0]);
                if (src.norm() < 1.f || src.norm() > 25.f)
                {
                    continue;
                }
                // Filter robots.
                bool remove = false;
                for (Index i = 0; i + 2 < robots.size(); i += 3)
                {
                    if ((src - ConstVec3Map(&robots[i])).norm() < 1.f)
                    {
                        remove = true;
                        break;
                    }
                }
                if (remove)
                {
                    continue;
                }
                Vec3Map dst(it_points_dst);
                dst = transform * src;
                it_points_dst += 3;
                ++n_added;
            }
            Index min_pts = 16;
            if (n_added < min_pts)
            {
                ROS_INFO("Discarding input cloud: not enough points to merge: %u < %u.", n_added, min_pts);
                return;
            }
            ROS_DEBUG("%lu / %lu points kept by distance and robot filters.",
                      size_t(n_added), size_t(n_pts));
            flann::Matrix<Elem> points(points_buf.begin(), n_added, 3);

            {
                Lock lock(map_.dirty_mutex_);
                map_.merge(points, origin_mat);

                map_.update_dirty();
//            ROS_INFO("Input cloud with %u points merged: %.3f s.", n_added, t.seconds_elapsed());
                // TODO: Mark affected map points for update?

                Timer t_send;
                sensor_msgs::PointCloud2 map_dirty;
                map_dirty.header.frame_id = map_frame_;
                map_dirty.header.stamp = cloud->header.stamp;
                map_.create_dirty_cloud(map_dirty);
                map_dirty_pub_.publish(map_dirty);
                ROS_DEBUG("Sending dirty cloud: %.3f s.", t_send.seconds_elapsed());

                map_.clear_dirty();
            }

            if (map_pub_.getNumSubscribers() > 0)
            {
                Timer t_send;
                sensor_msgs::PointCloud2 map_cloud;
                map_cloud.header.frame_id = map_frame_;
                map_cloud.header.stamp = cloud->header.stamp;
                map_.create_cloud(map_cloud);
                map_pub_.publish(map_cloud);
                ROS_DEBUG("Sending map: %.3f s.", t_send.seconds_elapsed());
            }
        }

    protected:
        typedef std::recursive_mutex Mutex;
        typedef std::lock_guard<Mutex> Lock;

        ros::NodeHandle& nh_;
        ros::NodeHandle& pnh_;
        std::unique_ptr<tf2_ros::Buffer> tf_;
        std::unique_ptr<tf2_ros::TransformListener> tf_sub_;
        ros::Publisher normal_label_cloud_pub_;
        ros::Publisher final_label_cloud_pub_;
        ros::Publisher path_cost_cloud_pub_;
        ros::Publisher utility_cloud_pub_;
        ros::Publisher final_cost_cloud_pub_;

        ros::Publisher path_pub_;
        ros::Publisher minpos_path_pub_;
        ros::Publisher viewpoints_pub_;
        ros::Publisher other_viewpoints_pub_;
        ros::Subscriber cloud_sub_;

        std::vector<ros::Subscriber> input_cloud_subs_;
        ros::Publisher map_pub_;
        ros::Publisher map_dirty_pub_;
        ros::Publisher map_diff_pub_;
        ros::Timer planning_timer_;
        ros::ServiceServer get_plan_service_;
        Mutex last_request_mutex_;
        nav_msgs::GetPlanRequest last_request_;
        ros::Timer viewpoints_update_timer_;
        ros::WallTimer update_params_timer_;

        std::string position_name_;
        std::string normal_name_;

        std::string map_frame_;
        std::string robot_frame_;
        std::map<std::string, std::string> robot_frames_;
        float max_cloud_age_;
        float max_pitch_;
        float max_roll_;
        int empty_ratio_;
        bool filter_robots_;

        int neighborhood_knn_;
        float neighborhood_radius_;
        int min_normal_pts_;
        float normal_radius_;

        float max_nn_height_diff_;
        float clearance_low_;
        float clearance_high_;
        float min_points_obstacle_;
        float max_ground_diff_std_;
        float max_ground_abs_diff_mean_;
        float edge_min_centroid_offset_;
        float min_dist_to_obstacle_;

        Mutex viewpoints_mutex_;
        float viewpoints_update_freq_;
        std::vector<Value> viewpoints_;  // 3xN
        std::vector<Value> other_viewpoints_;  // 3xN
//        std::map<std::string, Vec3> last_positions_;
        float min_vp_distance_;
        float max_vp_distance_;
        float self_factor_;
        float planning_freq_;

        Mutex initialized_mutex_;
        bool initialized_;

        int queue_size_;
        Mutex map_mutex_;
        Map map_;
//        Parameters params_;
    };

}  // namespace naex

#endif //NAEX_PLANNER_H
