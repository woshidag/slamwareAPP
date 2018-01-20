/**
* feature.h
* The Motion Planner feature
*
* Created By Tony Huang @ 2014-5-22
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/feature.h>
#include <rpos/core/pose.h>
#include <boost/optional.hpp>
#include <rpos/core/geometry.h>
#include "move_action.h"

namespace rpos {
    namespace features {

        namespace detail {
            class MotionPlannerImpl;
        }

        namespace motion_planner {

            enum MoveOptionFlag
            {
                MoveOptionFlagNone = 0,
                MoveOptionFlagAppending = 1,
                MoveOptionFlagMilestone = 2,
                MoveOptionFlagNoSmooth  = 4,
                MoveOptionFlagKeyPoints = 8,
                MoveOptionFlagPrecise   = 16,
                MoveOptionFlagWithYaw   = 32
            };

            struct RPOS_CORE_API MoveOptions
            {
                MoveOptions();

                MoveOptionFlag flag;
                boost::optional<double> speed_ratio;
            };

        }

        class RPOS_CORE_API MotionPlanner : public rpos::core::Feature{
        public:
            typedef detail::MotionPlannerImpl impl_t;

            RPOS_OBJECT_CTORS_WITH_BASE(MotionPlanner, rpos::core::Feature);
            MotionPlanner(boost::shared_ptr<detail::MotionPlannerImpl> impl);
            ~MotionPlanner();

        public:
            rpos::actions::MoveAction moveTo(const std::vector<rpos::core::Location>& locations, bool appending, bool isMilestone);
            rpos::actions::MoveAction moveTo(const rpos::core::Location& location, bool appending, bool isMilestone);
            rpos::actions::MoveAction moveTo(const std::vector<rpos::core::Location>& locations, const motion_planner::MoveOptions& options, float yaw);
            rpos::actions::MoveAction moveTo(const rpos::core::Location& location, const motion_planner::MoveOptions& options, float yaw);
            rpos::actions::MoveAction moveBy(const rpos::core::Direction& direction);
            rpos::actions::MoveAction moveBy(const rpos::core::Direction& direction, const motion_planner::MoveOptions& options);
            rpos::actions::MoveAction moveBy(float theta, const motion_planner::MoveOptions& options);
            rpos::actions::MoveAction rotateTo(const rpos::core::Rotation& orientation);
            rpos::actions::MoveAction rotateTo(const rpos::core::Rotation& orientation, const motion_planner::MoveOptions& options);
            rpos::actions::MoveAction rotate(const rpos::core::Rotation& rotation);
            rpos::actions::MoveAction rotate(const rpos::core::Rotation& rotation, const motion_planner::MoveOptions& options);
            rpos::actions::MoveAction recoverLocalization(const core::RectangleF& area);
            rpos::actions::MoveAction getCurrentAction();

            rpos::features::motion_planner::Path searchPath(const rpos::core::Location& location);
            rpos::features::motion_planner::Path getRobotTrack(int count);
        };

    }
}
