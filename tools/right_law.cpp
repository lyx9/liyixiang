 Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 ***********************************************************************/
/**
 * @file right_turn_law_test.cpp
 **/
#include <gtest/gtest.h>
#include "util/data_center.h"
#include "common/data/planning_data.h"
#include "traffic_law/right_turn_law.h"
#include "core/planner.h"
#include "cybertron/cybertron.h"
#include "common/houston_gflags.h"
#include "common/logger.h"
#include "task/task.h"
#include "new_router.pb.h"
namespace adu {
namespace planning {
class RightTurnLawTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        _module_root = cybertron::ModuleRoot();
        std::fstream input_file(_module_root + "test_data/config.pb.txt", std::ios::in);
        CHECK(input_file.good());
        std::string config_data = std::string(std::istreambuf_iterator<char>(input_file),
                std::istreambuf_iterator<char>());
        CHECK(!config_data.empty()) << "Please check if" << FLAGS_houston_adu_configure_file
                << "is empty" << std::endl;
        CHECK(::google::protobuf::TextFormat::ParseFromString(config_data, &_veh_config))
                << "Fail to parse" << FLAGS_houston_adu_configure_file << "to proto format."
                << "Please check if the format of the file is identical to the defined proto"
                << std::endl;
        FLAGS_houston_enable_align_prediction_timestamp = false;
        FLAGS_houston_enable_prepare_change_lane = true;
        FLAGS_houston_prepare_change_lane_min_distance = 200;
        FLAGS_houston_planning_gravity_center_as_init_position = true;
        FLAGS_houston_adu_data_path = _module_root + "test_data/";
        FLAGS_houston_adu_map_data = "yizhuang_d_map.bin";
        FLAGS_houston_adu_configure_file = "config.pb.txt";
    }
    
    void TearDown() {
        cybertron::Shutdown();
    }
protected:
    void mock_carstatus(const ::adu::common::math::Vec2d& init_position,
        const double init_v, const uint32_t seq_num, const double time_sec);
    void mock_routing();
    void mock_object_table();
protected:
    std::string _module_root;
    ::adu::common::config::Config _veh_config;
    ::adu::common::new_router::RoutingResponse _routing_res;
};
void RightTurnLawTest::mock_routing() {
    auto header = _routing_res.mutable_header();
    header->set_timestamp_sec(0.0);
    header->set_sequence_num(1);
    ::adu::common::new_router::RoutingRequest* request = _routing_res.mutable_routing_request();
    ::adu::common::new_router::LaneWaypoint* start_waypoint = request->add_waypoint();
    start_waypoint->set_id("59892_1_-5");
    start_waypoint->set_s(0.0);
    ::adu::common::PointENU* start_pose = start_waypoint->mutable_pose();
    start_pose->set_x(455368.0391);
    start_pose->set_y(4404869.6671);
    start_pose->set_z(0.0);
    ::adu::common::new_router::LaneWaypoint* end_waypoint = request->add_waypoint();
    end_waypoint->set_id("60132_1_-2");
    end_waypoint->set_s(24.146);
    ::adu::common::PointENU* end_pose = end_waypoint->mutable_pose();
    end_pose->set_x(455394.31);
    end_pose->set_y(4404910.27);
    end_pose->set_z(0.0);
    auto request_header = request->mutable_header();
    request_header->set_timestamp_sec(0.0);
    ::adu::common::new_router::RoadSegment* road_segment_0 = _routing_res.add_road();
    road_segment_0->set_id("0");
    ::adu::common::new_router::Passage* passage_0_0 = road_segment_0->add_passage();
    passage_0_0->set_id("0_0");
    passage_0_0->set_can_exit(true);
    /*
    auto segment_0_0_0 = passage_0_0->add_segment();
    segment_0_0_0->set_id("60136_1_-5");
    segment_0_0_0->set_start_s(0.16304);
    segment_0_0_0->set_end_s(13.78191);
    
    auto segment_0_0_1 = passage_0_0->add_segment();
    segment_0_0_1->set_id("60138_1_-5");
    segment_0_0_1->set_start_s(0.0);
    segment_0_0_1->set_end_s(7.442);
    
    auto segment_0_0_2 = passage_0_0->add_segment();
    segment_0_0_2->set_id("65189_1_-5");
    segment_0_0_2->set_start_s(0.0);
    segment_0_0_2->set_end_s(1.498);
    
    auto segment_0_0_3 = passage_0_0->add_segment();
    segment_0_0_3->set_id("62825_1_-5");
    segment_0_0_3->set_start_s(0.0);
    segment_0_0_3->set_end_s(1.41379);
    */
    auto segment_0_0_4 = passage_0_0->add_segment();
    segment_0_0_4->set_id("59892_1_-5");
    segment_0_0_4->set_start_s(0.0);
    segment_0_0_4->set_end_s(4.83239);
    
    auto segment_1_1_0 = passage_0_0->add_segment();
    segment_1_1_0->set_id("60222a_1_-1");
    segment_1_1_0->set_start_s(0.0);
    segment_1_1_0->set_end_s(25.57628);
    
    auto segment_1_1_1 = passage_0_0->add_segment();
    segment_1_1_1->set_id("60132_1_-2");
    segment_1_1_1->set_start_s(0.0);
    segment_1_1_1->set_end_s(24.146);
    DataCenter::instance()->mutable_environment()->set_router(_routing_res);
    //std::cout << "routing pb:" << _routing_res << std::endl;
}
void RightTurnLawTest::mock_carstatus(const ::adu::common::math::Vec2d& init_position,
        const double init_v, const uint32_t seq_num, const double time_sec) {
    // mock car status
    ::adu::common::status::Status status;
    std::cout << "int x: " << init_position.x() << " y: " << init_position.y() << std::endl;
    status.mutable_pose()->mutable_position()->set_x(init_position.x());
    status.mutable_pose()->mutable_position()->set_y(init_position.y());
    status.mutable_pose()->mutable_position()->set_z(0);
    status.mutable_pose()->mutable_orientation()->set_qx(0);
    status.mutable_pose()->mutable_orientation()->set_qy(0);
    status.mutable_pose()->mutable_orientation()->set_qz(-0.70710678);
    status.mutable_pose()->mutable_orientation()->set_qw(0.70710678);
    status.mutable_vehicle_frame()->mutable_position()->set_x(init_position.x());
    status.mutable_vehicle_frame()->mutable_position()->set_y(init_position.y());
    status.mutable_vehicle_frame()->mutable_position()->set_z(0.0);
    status.mutable_vehicle_frame()->set_yaw(-1.0573);
    status.mutable_header()->set_timestamp_sec(time_sec);
    status.mutable_header()->set_sequence_num(seq_num);
    status.mutable_pose()->mutable_linear_velocity()->set_x(init_v * cos(-1.0143));
    status.mutable_pose()->mutable_linear_velocity()->set_y(init_v * sin(-1.0143));
    status.mutable_pose()->mutable_linear_velocity()->set_z(0.0);
    status.mutable_chassis()->set_steering_percentage(1);
    status.mutable_chassis()->set_driving_mode(::adu::common::status::COMPLETE_AUTO_DRIVE);
    DataCenter::instance()->mutable_environment()
                          ->set_vehicle_state(_veh_config, status);
}
void RightTurnLawTest::mock_object_table() {
    std::list<std::unique_ptr<PlanningObstacle>> obstacle_list;
    std::unique_ptr<PlanningObstacle> dynamic_obstacle_ptr(new PlanningObstacle());
    std::vector<::adu::common::math::Vec2d> dynamic_vec_points;
    ::adu::common::math::Vec2d xy_point_1(455315.604, 4404860.519);
    ::adu::common::math::Vec2d xy_point_2(455320.604, 4404862.519);
    ::adu::common::math::Vec2d xy_point_3(455319.604, 4404866.519);
    ::adu::common::math::Vec2d xy_point_4(455314.604, 4404864.519);
    ::adu::common::math::Vec2d central_xy_point(455318.4558, 4404863.944);
    dynamic_vec_points.emplace_back(xy_point_1);
    dynamic_vec_points.emplace_back(xy_point_2);
    dynamic_vec_points.emplace_back(xy_point_3);
    dynamic_vec_points.emplace_back(xy_point_4);
    ::adu::common::math::Polygon2d dynamic_polygon(dynamic_vec_points);
    *dynamic_obstacle_ptr->mutable_polygon() = dynamic_polygon;
    dynamic_obstacle_ptr->set_id(10000);
    dynamic_obstacle_ptr->set_height(2.0);
    dynamic_obstacle_ptr->set_length(6.0);
    dynamic_obstacle_ptr->set_width(3.0);
    dynamic_obstacle_ptr->set_speed(9.0);
    dynamic_obstacle_ptr->set_type(Obstacle::ObstacleType::VEHICLE);
    dynamic_obstacle_ptr->set_center(central_xy_point);
    std::vector<TrajectoryPoint> prediction_trajectory_points;
    ::adu::common::math::Vec2d pred_end_xy(455375.567, 4404894.7258);
    //double step_x = (pred_end_xy.x() - central_xy_point.x()) / 80;
    //double step_y = (pred_end_xy.y() - central_xy_point.y()) / 80;
    for (std::size_t i = 0; i < 81; ++i) {
        //double cur_y = central_xy_point.y() + step_y * i;
        //double cur_x = central_xy_point.x() + step_x * i;
        double heading = 0.49434;
        double cur_x = central_xy_point.x() + dynamic_obstacle_ptr->speed() * i * 0.1 * cos(heading);
        double cur_y = central_xy_point.y() + dynamic_obstacle_ptr->speed() * i * 0.1 * sin(heading);
        double trajectory_s = dynamic_obstacle_ptr->speed() * i * 0.1;
        ::adu::common::math::Vec2d cur_xy_point(cur_x, cur_y);
        PathPoint path_point(cur_xy_point, heading, 0.0, 0.0, 0.0, trajectory_s);
        prediction_trajectory_points.emplace_back(path_point, dynamic_obstacle_ptr->speed(), 0.0, i * 0.1);
    }
    PredictionTrajectory prediction_trajectory;
    prediction_trajectory.set_trajectory_points(prediction_trajectory_points);
    dynamic_obstacle_ptr->add_prediction_trajectory(prediction_trajectory);
    
    std::unique_ptr<PlanningObstacle> dynamic_obstacle_ptr_1(new PlanningObstacle());
    std::vector<::adu::common::math::Vec2d> dynamic_vec_points_1;
    ::adu::common::math::Vec2d xy_point_1_1(455315.604 + 20.0 * cos(0.49434), 4404860.519 + 20.0 * sin(0.49434));
    ::adu::common::math::Vec2d xy_point_1_2(455320.604 + 20.0 * cos(0.49434), 4404862.519 + 20.0 * sin(0.49434));
    ::adu::common::math::Vec2d xy_point_1_3(455319.604 + 20.0 * cos(0.49434), 4404866.519 + 20.0 * sin(0.49434));
    ::adu::common::math::Vec2d xy_point_1_4(455314.604 + 20.0 * cos(0.49434), 4404864.519 + 20.0 * sin(0.49434));
    ::adu::common::math::Vec2d central_xy_point_1(455318.4558 + 20.0 * cos(0.49434), 4404863.944 + 20.0 * sin(0.49434));
    dynamic_vec_points_1.emplace_back(xy_point_1_1);
    dynamic_vec_points_1.emplace_back(xy_point_1_2);
    dynamic_vec_points_1.emplace_back(xy_point_1_3);
    dynamic_vec_points_1.emplace_back(xy_point_1_4);
    ::adu::common::math::Polygon2d dynamic_polygon_1(dynamic_vec_points_1);
    *dynamic_obstacle_ptr_1->mutable_polygon() = dynamic_polygon_1;
    dynamic_obstacle_ptr_1->set_id(10001);
    dynamic_obstacle_ptr_1->set_height(2.0);
    dynamic_obstacle_ptr_1->set_length(2.0);
    dynamic_obstacle_ptr_1->set_width(2.0);
    dynamic_obstacle_ptr_1->set_speed(8.0);
    dynamic_obstacle_ptr_1->set_type(Obstacle::ObstacleType::BICYCLE);
    dynamic_obstacle_ptr_1->set_center(central_xy_point_1);
    std::vector<TrajectoryPoint> prediction_trajectory_points_1;
    for (std::size_t i = 0; i < 51; ++i) {
        double heading = 0.49434;
        double cur_x = central_xy_point_1.x() + dynamic_obstacle_ptr_1->speed() * i * 0.1 * cos(heading);
        double cur_y = central_xy_point_1.y() + dynamic_obstacle_ptr_1->speed() * i * 0.1 * sin(heading);
        double trajectory_s = dynamic_obstacle_ptr_1->speed() * i * 0.1;
        ::adu::common::math::Vec2d cur_xy_point(cur_x, cur_y);
        PathPoint path_point(cur_xy_point, heading, 0.0, 0.0, 0.0, trajectory_s);
        prediction_trajectory_points_1.emplace_back(path_point, dynamic_obstacle_ptr_1->speed(), 0.0, i * 0.1);
    }
    PredictionTrajectory prediction_trajectory_1;
    prediction_trajectory_1.set_trajectory_points(prediction_trajectory_points_1);
    dynamic_obstacle_ptr_1->add_prediction_trajectory(prediction_trajectory_1);
    
    std::unique_ptr<PlanningObstacle> dynamic_obstacle_ptr_2(new PlanningObstacle());
    std::vector<::adu::common::math::Vec2d> dynamic_vec_points_2;
    ::adu::common::math::Vec2d xy_point_2_1(455315.604 + 40.0 * cos(0.49434), 4404860.519 + 40.0 * sin(0.49434));
    ::adu::common::math::Vec2d xy_point_2_2(455320.604 + 40.0 * cos(0.49434), 4404862.519 + 40.0 * sin(0.49434));
    ::adu::common::math::Vec2d xy_point_2_3(455319.604 + 40.0 * cos(0.49434), 4404866.519 + 40.0 * sin(0.49434));
    ::adu::common::math::Vec2d xy_point_2_4(455314.604 + 40.0 * cos(0.49434), 4404864.519 + 40.0 * sin(0.49434));
    ::adu::common::math::Vec2d central_xy_point_2(455318.4558 + 40.0 * cos(0.49434), 4404863.944 + 40.0 * sin(0.49434));
    dynamic_vec_points_2.emplace_back(xy_point_2_1);
    dynamic_vec_points_2.emplace_back(xy_point_2_2);
    dynamic_vec_points_2.emplace_back(xy_point_2_3);
    dynamic_vec_points_2.emplace_back(xy_point_2_4);
    ::adu::common::math::Polygon2d dynamic_polygon_2(dynamic_vec_points_2);
    *dynamic_obstacle_ptr_2->mutable_polygon() = dynamic_polygon_2;
    dynamic_obstacle_ptr_2->set_id(10002);
    dynamic_obstacle_ptr_2->set_height(1.7);
    dynamic_obstacle_ptr_2->set_length(1.0);
    dynamic_obstacle_ptr_2->set_width(1.0);
    dynamic_obstacle_ptr_2->set_speed(4.0);
    dynamic_obstacle_ptr_2->set_type(Obstacle::ObstacleType::PEDESTRIAN);
    dynamic_obstacle_ptr_2->set_center(central_xy_point_2);
    std::vector<TrajectoryPoint> prediction_trajectory_points_2;
    for (std::size_t i = 0; i < 31; ++i) {
        double heading = 0.49434;
        double cur_x = central_xy_point_2.x() + dynamic_obstacle_ptr_2->speed() * i * 0.1 * cos(heading);
        double cur_y = central_xy_point_2.y() + dynamic_obstacle_ptr_2->speed() * i * 0.1 * sin(heading);
        double trajectory_s = dynamic_obstacle_ptr_2->speed() * i * 0.1;
        ::adu::common::math::Vec2d cur_xy_point(cur_x, cur_y);
        PathPoint path_point(cur_xy_point, heading, 0.0, 0.0, 0.0, trajectory_s);
        prediction_trajectory_points_2.emplace_back(path_point, dynamic_obstacle_ptr_2->speed(), 0.0, i * 0.1);
    }
    PredictionTrajectory prediction_trajectory_2;
    prediction_trajectory_2.set_trajectory_points(prediction_trajectory_points_2);
    dynamic_obstacle_ptr_2->add_prediction_trajectory(prediction_trajectory_2);
    
    obstacle_list.push_back(std::move(dynamic_obstacle_ptr));
    obstacle_list.push_back(std::move(dynamic_obstacle_ptr_1));
    obstacle_list.push_back(std::move(dynamic_obstacle_ptr_2));
    DataCenter::instance()->inject_obstacle(2, obstacle_list);
    DataCenter::instance()->update_object_table();
}
TEST_F(RightTurnLawTest, inner_lane_yield) {
    double init_v = 8;
    ::adu::common::math::Vec2d init_position_0{455368.0391, 4404869.6671};
    ::adu::common::math::Vec2d init_position_1{455365.6656, 4404873.8764};
    ::adu::common::math::Vec2d init_position{0.0, 0.0};
    double adc_heading = -1.0573;
    const double duration = 0.1;
    auto data_center = DataCenter::instance();
    /*
    ::adu::common::traffic_light::TrafficLightDetection empty_traffic_light;
    empty_traffic_light.mutable_header()->set_sequence_num(1);
    ::adu::common::perception::PerceptionObstacles empty_perception;
    auto perception_obstacle = empty_perception.add_perception_obstacle();
    perception_obstacle->set_id(10000);
    empty_perception.mutable_header()->set_sequence_num(1);
    data_center->mutable_environment()->set_perception(empty_perception);
    data_center->mutable_environment()->set_traffic_light(empty_traffic_light);
    mock_carstatus(init_position_0, init_v, 1, 0.0);
    mock_routing();
    DataCenter::instance()->init_frame(1);
    ErrorCode ret = Planner::instance()->make_plan();
    EXPECT_EQ(ret, ErrorCode::PLANNING_OK);
    DataCenter::instance()->save_frame();
    */
    ::adu::common::traffic_light::TrafficLightDetection empty_traffic_light;
    ::adu::common::perception::PerceptionObstacles perception;
    auto perception_obstacle_0 = perception.add_perception_obstacle();
    perception_obstacle_0->set_id(10000);
    auto perception_obstacle_1 = perception.add_perception_obstacle();
    perception_obstacle_1->set_id(10001);
    auto perception_obstacle_2 = perception.add_perception_obstacle();
    perception_obstacle_2->set_id(10002);
    mock_object_table();
    mock_routing();
    
    // satisfy continuous need yield condition
    uint32_t cur_seq = 0;
    double cur_time_sec = 0.0;
    ErrorCode ret = ErrorCode::PLANNING_OK;
    for (std::size_t i = 0; i < 6; ++i) {
        cur_seq += 1;
        empty_traffic_light.mutable_header()->set_sequence_num(cur_seq);
        perception.mutable_header()->set_sequence_num(cur_seq);
        data_center->mutable_environment()->set_traffic_light(empty_traffic_light);
        data_center->mutable_environment()->set_perception(perception);
        init_position.set_x(init_position_0.x() + init_v * cur_time_sec * cos(adc_heading));
        init_position.set_y(init_position_0.y() + init_v * cur_time_sec * sin(adc_heading));
        mock_carstatus(init_position, init_v, cur_seq, cur_time_sec);
        data_center->init_frame(cur_seq);
        mock_object_table();
        ret = Planner::instance()->make_plan();
        EXPECT_EQ(ret, ErrorCode::PLANNING_OK);
        DataCenter::instance()->save_frame();
        cur_time_sec += duration;
    }
   
    // satisfy continuous not need yield condition
    init_v = 1;
    ::adu::common::perception::PerceptionObstacles empty_perception;
    for (std::size_t i = 0; i < 26; ++i) {
        cur_seq += 1;
        empty_traffic_light.mutable_header()->set_sequence_num(cur_seq);
        empty_perception.mutable_header()->set_sequence_num(cur_seq);
        data_center->mutable_environment()->set_traffic_light(empty_traffic_light);
        data_center->mutable_environment()->set_perception(empty_perception);
        init_position.set_x(init_position_0.x() + init_v * cur_time_sec * cos(adc_heading));
        init_position.set_y(init_position_0.y() + init_v * cur_time_sec * sin(adc_heading));
        mock_carstatus(init_position, init_v, cur_seq, cur_time_sec);
        data_center->init_frame(cur_seq);
        mock_object_table();
        ret = Planner::instance()->make_plan();
        EXPECT_EQ(ret, ErrorCode::PLANNING_OK);
        DataCenter::instance()->save_frame();
        cur_time_sec += duration;
    }
}
} //planning
} //adu
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
