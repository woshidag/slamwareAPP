#pragma once

#include <vector>

namespace rpos { namespace message { namespace depth_camera {

    /**
    * @brief Depth camera frame
    */
    struct DepthCameraFrame
    {
        /**
        * @brief Min distance of the sensor (in meters)
        */
        float minValidDistance;

        /**
        * @brief Max distance of the sensor (in meters)
        */
        float maxValidDistance;

        /**
        * @brief The pitch of the first line of the frame (in rad)
        */
        float minFovPitch;

        /**
        * @brief The pitch of the last line of the frame (in rad)
        */
        float maxFovPitch;

        /**
        * @brief The yaw of the last column of the frame (in rad)
        */
        float minFovYaw;

        /**
        * @brief The yaw of the first column of the frame (in rad)
        */
        float maxFovYaw;

        /**
        * @brief how many columns in this frame
        */
        int cols;

        /**
        * @brief how many rows in this frame
        */
        int rows;

        // TODO:
        // We now use std::vector to store data, which will make *copying* frames very expensive
        // we should use a data structure support CoW (copy on write) or just use shared pointer to store the internal data
        /**
        * @brief The data (order: row by row, cell by cell in each row)
        */
        std::vector<float> data;
    };

    struct FlattenDepthCameraScanPoint
    {
        float dist;     // in meter
        float angle;    // in degree, 0 expected to be the front of LIDAR, and increase by rotate in counter-clockwise (left-hand system)
    };

    typedef std::vector<FlattenDepthCameraScanPoint> FlattenDepthCameraScan;

}}}