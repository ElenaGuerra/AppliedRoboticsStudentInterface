#pragma once

class DubinsArc {

public:
    float x0;
    float y0;
    float th0;
    float k;
    float L;
    float xf;
    float yf;
    float thf;

    DubinsArc();
    DubinsArc(float x_0, float y_0, float th_0, float curvature, float length, float x_f, float y_f, float th_f);
};
