#include "condition_handler_base.h"
#include "map_lane.pb.h"
#include "math/polygon2d.h"
#include "common_geometry.pb.h"
#include "hdmap_common.h"
#include "common/map_service.h"
#include "util.h"
#include <gtest/gtest.h>
#include "sim_grading_config.pb.h"
#include "sim_grading_result.pb.h"
#include "grade.h"
#include "utils.h"
#include "sim_grading_system_gflags.h"
#include "map_service.h"
#include "scenario_service.h"
#include "json/json.h"

bool NudgeConditionJudge::obs_nudge(const SimulationWorld& world,
                                        const double front_dis,
                                        const double dv_th) {
    std::vector<Object> objects;
    bool ret2 = ConditionEvaluatorUtil::get_objects_in_front_adc(
                    world.auto_driving_car().position_x(),
                    world.auto_driving_car().position_y(),
                    world,
                    front_dis,
                    &objects,
                    true);
    if (!ret2 || objects.size() <= 0) {
        return false;
    }

    std::vector<Polygon2d> pred_adc_polygon_vec;
    const auto& cur_adc = world.auto_driving_car();
    if (cur_adc.map_data().has_lane_id() && cur_adc.map_data().has_lane_s()) {
        ::adu::common::hdmap::Id lane_id;
        lane_id.set_id(cur_adc.map_data().lane_id());
        ::adu::hdmap::LaneInfoConstPtr lane_info;
        bool success = SimMapSvr::get_lane_by_id(lane_id, &lane_info);
        if (!success) {
            return false;
        }
        ConditionEvaluatorUtil::get_pred_adc_polygon_vec(lane_info,
                        cur_adc.map_data().lane_s(),
                        front_dis,
                        &pred_adc_polygon_vec);
    }
    LOG(INFO) << "pred_adc_polygon_vec size: " << pred_adc_polygon_vec.size();

    ool DuringLaneChangeConditionHandler::is_adc_cross_line(const int idx, const double search_dis,
                                                                const int car_type) const {
        const auto& adc = world_list.at(idx).auto_driving_car();
        const auto& adc_pos = ConditionEvaluatorUtil::to_point_enu(adc.position_x(), adc.position_y());
        std::vector<Vec2d> adc_polygon_pts = ConditionEvaluatorUtil::get_adc_box_polygon(
                                                        adc_pos.x(), adc_pos.y(), adc.heading(), car_type);
        Polygon2d adc_polygon(adc_polygon_pts);
        return SimMapSvr::is_polygon_on_any_lane_boundary(adc_polygon, adc_pos, search_dis);
    }
    void DuringLaneChangeConditionHandler::check_adc_lane_change(
                        const int cur_index,
                        const int end_check_idx,
                        const DuringLaneChangeCondition& during_lane_change_condition) {
        const auto car_type = during_lane_change_condition.car_type();
        int i = 0;
        bool has_lane_change = false;
        for (i = cur_index; i<= end_check_idx; ++i) {
            if (ConditionEvaluatorUtil::is_adc_change_lane(world_list, i)) {
                LOG(INFO) << "lane change at index: " << i;
                has_lane_change = true;
                break;
            }
        }
        if (!has_lane_change) {
            _cur_checked_end_idx = end_check_idx;
            return;
        }
        // if found lane change
        double search_dis = std::sqrt(FLAGS_length * FLAGS_length + FLAGS_width * FLAGS_width) + 2;
        int j = 0;
        for (j = i - 1; j >= cur_index; --j) {
            if (!is_adc_cross_line(j, search_dis, car_type)) {
                break;
            }
        }

        _start_lane_change_idx = j + 1 ;

        int k = 0;
        for (k = i + 1; k < world_list.size(); ++k) {
            if (!is_adc_cross_line(k, search_dis, car_type)) {
                break;
            }
        }
        _end_lane_change_idx = k - 1;
        _cur_checked_end_idx = k - 1;
        return;
    }

bool NudgeConditionJudge::is_adc_nudging（const int &a, const int &b）
{
  #todo
}

bool NudgeConditionHandler::eval_nudge()
{

  #todo
}

bool NudgeConditionJudge::NotOnRealline(const &adc)
{

  #TODO

}
float NudgeConditionJudge::eval_nudge(const NudgeCondition& nudge_condition,
                                       const int cur_index,
                                       ConditionResult *ptr_frame_result,
                                       ConditionResult *ptr_overall_result,
                                       const int start_detail_result_index) {
    LOG(INFO) << "_nudge_status: " << _nudge_status;
    return is_adc_nudging(nudge_condition, cur_index);
}
