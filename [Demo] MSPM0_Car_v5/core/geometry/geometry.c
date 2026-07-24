#include "geometry.h"

#include "kinematics/kinematics.h"

#include <math.h>
#include <stddef.h>

CORE_POINT2F Geometry_Lerp2D(CORE_POINT2F start,
                             CORE_POINT2F end,
                             float ratio){
    CORE_POINT2F point = {
        .x = start.x + (end.x - start.x) * ratio,
        .y = start.y + (end.y - start.y) * ratio,
    };

    return point;
}

CORE_POINT2F Geometry_RectBilinearInterpolate(const GEOMETRY_RECT2F *rect,
                                              float u,
                                              float v){
    CORE_POINT2F point = {0.0f, 0.0f};

    if (rect == NULL){
        return point;
    }

    CORE_POINT2F top = Geometry_Lerp2D(rect->top_left, rect->top_right, u);
    CORE_POINT2F bottom = Geometry_Lerp2D(rect->bottom_left, rect->bottom_right, u);

    return Geometry_Lerp2D(top, bottom, v);
}

CORE_POINT2F Geometry_PaperToRectPoint(CORE_POINT2F paper_point,
                                       float paper_width,
                                       float paper_height,
                                       const GEOMETRY_RECT2F *rect){
    CORE_POINT2F point = {0.0f, 0.0f};

    if ((rect == NULL) || (paper_width == 0.0f) || (paper_height == 0.0f)){
        return point;
    }

    float u = paper_point.x / paper_width;
    float v = paper_point.y / paper_height;

    return Geometry_RectBilinearInterpolate(rect, u, v);
}

CORE_POINT2F Geometry_CirclePointDeg(CORE_POINT2F center,
                                     float radius,
                                     float angle_deg){
    float angle_rad = KINEMATICS_DEG_TO_RAD(angle_deg);
    CORE_POINT2F point = {
        .x = center.x + radius * cosf(angle_rad),
        .y = center.y + radius * sinf(angle_rad),
    };

    return point;
}

void Geometry_RectFromArray(const uint16_t rect_data[8], GEOMETRY_RECT2F *rect){
    if ((rect_data == NULL) || (rect == NULL)){
        return;
    }

    rect->bottom_left.x = (float)rect_data[0];
    rect->bottom_left.y = (float)rect_data[1];
    rect->bottom_right.x = (float)rect_data[2];
    rect->bottom_right.y = (float)rect_data[3];
    rect->top_right.x = (float)rect_data[4];
    rect->top_right.y = (float)rect_data[5];
    rect->top_left.x = (float)rect_data[6];
    rect->top_left.y = (float)rect_data[7];
}
