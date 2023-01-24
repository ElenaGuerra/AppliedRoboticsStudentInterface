#pragma once

#include "DubinsArc.hpp"

class DubinsStructure {
public:
    DubinsArc a1;
    DubinsArc a2;
    DubinsArc a3;
    float L;
    int manoeuvre;
};
