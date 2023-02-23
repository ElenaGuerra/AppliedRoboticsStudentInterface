#include "Dubins.hpp"

DubinsArc::DubinsArc(){
  DubinsArc(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}
DubinsArc::DubinsArc(float x_0, float y_0, float th_0, float curvature, float length, float x_f, float y_f, float th_f){
  x0= x_0;
  y0= y_0;
  th0=th_0;

  xf= x_f;
  yf= y_f;
  thf=th_f;

  k = curvature;
  L=length;
}

Dubins::Dubins(obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr obstacles_msg){
  obstacles = obstacles_msg;
}

DubinsStructure Dubins::dubins_shortest_path(float x0, float y0, float th0, float xf, float yf, float thf, float Kmax) {

    float dx = xf - x0;
    float dy = yf - y0;
    float phi = atan2(dy, dx);
    float lambda = hypot(dx, dy) / 2;

    float sc_th0 = mod2pi(th0 - phi);
    float sc_thf = mod2pi(thf - phi);
    float sc_Kmax = Kmax * lambda;

    int pidx = -1;
    float L = 0;
    bool firstValidPath=true;

    DubinsStructure final_curve;

    float sc_s1 = 0;
    float sc_s2 = 0;
    float sc_s3 = 0;

    for (int i = 0; i < 6; i++) {
        float sc_s1_c = 0;
        float sc_s2_c = 0;
        float sc_s3_c = 0;
        bool ok = primitives(i, sc_th0, sc_thf, sc_Kmax, &sc_s1_c, &sc_s2_c, &sc_s3_c);

        DubinsStructure curve = dubinscurve(x0, y0, th0, sc_s1_c*lambda, sc_s2_c*lambda, sc_s3_c*lambda, ksigns[i][0] * Kmax, ksigns[i][1] * Kmax, ksigns[i][2] * Kmax);
        //ok = ok && !checkIntersection(curve);

        float Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
        if (ok && (Lcur < L || firstValidPath)) {
            firstValidPath = false;
            L = Lcur;
            sc_s1 = sc_s1_c;
            sc_s2 = sc_s2_c;
            sc_s3 = sc_s3_c;
            pidx = i;
            final_curve = curve;
        }
    }

    return final_curve;
};

void Dubins::dubins_full_path(float originalTh, rclcpp::Time stamp, list<geometry_msgs::msg::Point> pathToFollow, std::vector<geometry_msgs::msg::PoseStamped>& pathList){

  float currentTh = originalTh;
  if(pathToFollow.size()==1){
    addSinglePoints(pathList, stamp, pathToFollow.front().x, pathToFollow.front().y, originalTh);
  }
  else if(pathToFollow.size()==2){
    DubinsStructure temp = dubins_shortest_path(pathToFollow.front().x, pathToFollow.front().y, originalTh, pathToFollow.back().x, pathToFollow.back().y, originalTh, 3);

    addArcWithIntrapoints(pathList, stamp, temp.a1, 50, true);
    addArcWithIntrapoints(pathList, stamp, temp.a2, 50, false);
    addArcWithIntrapoints(pathList, stamp, temp.a3, 50, false);
  }
  else if(pathToFollow.size()>2){

    geometry_msgs::msg::Point prevPoint;
    geometry_msgs::msg::Point prevprevPoint;

    bool firstPoint = true;
    bool atleastSecondPoint = false;
    bool atleastThirdPoint = false;

    for(geometry_msgs::msg::Point node : pathToFollow){
      if(atleastSecondPoint){
        if(atleastThirdPoint){
          float targetTh = chooseEndingOrientation(prevPoint.x, prevPoint.y, node.x, node.y);
          DubinsStructure temp = dubins_shortest_path(prevprevPoint.x, prevprevPoint.y, currentTh, prevPoint.x, prevPoint.y, targetTh, 3);

          addArcWithIntrapoints(pathList, stamp, temp.a1, 50, firstPoint);
          addArcWithIntrapoints(pathList, stamp, temp.a2, 50, false);
          addArcWithIntrapoints(pathList, stamp, temp.a3, 50, false);
          firstPoint = false;
        }
        atleastThirdPoint = true;
        prevprevPoint = prevPoint;
      }
      atleastSecondPoint=true;
      prevPoint = node;
    }
    // Last node is calculated outside the LOOP
    // (prevprevPoint = second to last point, prevPoint = last point, node = NULL)
    DubinsStructure temp = dubins_shortest_path(prevprevPoint.x, prevprevPoint.y, currentTh, prevPoint.x, prevPoint.y, currentTh, 3);

    addArcWithIntrapoints(pathList, stamp, temp.a1, 100, false);
    addArcWithIntrapoints(pathList, stamp, temp.a2, 100, false);
    addArcWithIntrapoints(pathList, stamp, temp.a3, 100, false);

  }
}


float Dubins::mod2pi(float ang) {
    float out = ang;
    while (out < 0) {
        out = out + 2 * M_PI;
    }
    while (out >= 2 * M_PI) {
        out = out - 2 * M_PI;
    }
    return out;
};

bool Dubins::primitives(int type, float sc_th0, float sc_thf, float sc_Kmax, float* sc_s1, float* sc_s2, float* sc_s3) {
    float invK = 1 / sc_Kmax;
    bool ok = false;

    float C = 0;
    float S = 0;
    float temp1 = 0;
    float temp2 = 0;
    float temp3 = 0;
    std::string ok_str ="";

    switch (type) {
        // LSL
    case 0:
        C = cos(sc_thf) - cos(sc_th0);
        S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
        temp1 = atan2(C, S);
        *sc_s1 = invK * mod2pi(temp1 - sc_th0);
        temp2 = 2 + 4 * pow(sc_Kmax, 2) - 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
        if (temp2 < 0) {
            ok = false;
            *sc_s1 = 0;
            *sc_s2 = 0;
            *sc_s3 = 0;
        }
        else {
            *sc_s2 = invK * sqrt(temp2);
            *sc_s3 = invK * mod2pi(sc_thf- temp1);
            ok = true;
        }

        //ok_str = ok?"true":"false";
        //std::cout << "primitives(): LSL ("<< ok_str <<"): " << std::to_string(*sc_s1) << ", " << std::to_string(*sc_s2) << ", " << std::to_string(*sc_s3) << ": " << std::to_string(*sc_s1+ *sc_s2 + *sc_s3) << std::endl;
        break;
        // RSR
    case 1:
        C = cos(sc_th0) - cos(sc_thf);
        S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
        temp1 = atan2(C, S);
        *sc_s1 = invK * mod2pi(sc_th0 - temp1);
        temp2 = 2 + 4 * pow(sc_Kmax, 2) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
        if (temp2 < 0) {
            ok = false;
            *sc_s1 = 0;
            *sc_s2 = 0;
            *sc_s3 = 0;
        }
        else {
            *sc_s2 = invK * sqrt(temp2);
            *sc_s3 = invK * mod2pi(temp1 - sc_thf);
            ok = true;
        }
        //ok_str = ok?"true":"false";
        //std::cout << "primitives(): RSR ("<< ok_str <<"): " << std::to_string(*sc_s1) << ", " << std::to_string(*sc_s2) << ", " << std::to_string(*sc_s3) << ": " << std::to_string(*sc_s1+ *sc_s2 + *sc_s3) << std::endl;
        break;
        // LSR
    case 2:
        C = cos(sc_th0) + cos(sc_thf);
        S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);
        temp1 = atan2(-C, S);
        temp3 = 4 * pow(sc_Kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
        if (temp3 < 0) {
            ok = false;
            *sc_s1 = 0;
            *sc_s2 = 0;
            *sc_s3 = 0;
        }
        else {
            ok = true;
            *sc_s2 = invK * sqrt(temp3);
            temp2 = -atan2(-2, *sc_s2 * sc_Kmax);
            *sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
            *sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);
        }
        //ok_str = ok?"true":"false";
        //std::cout << "primitives(): LSR ("<< ok_str <<"): " << std::to_string(*sc_s1) << ", " << std::to_string(*sc_s2) << ", " << std::to_string(*sc_s3) << ": " << std::to_string(*sc_s1+ *sc_s2 + *sc_s3) << std::endl;
        break;

        // RSL
    case 3:
        C = cos(sc_th0) + cos(sc_thf);
        S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);
        temp1 = atan2(C, S);
        temp3 = 4 * pow(sc_Kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
        if (temp3 < 0) {
            ok = false;
            *sc_s1 = 0;
            *sc_s2 = 0;
            *sc_s3 = 0;
        }
        else {
            ok = true;
            *sc_s2 = invK * sqrt(temp3);
            temp2 = atan2(2, *sc_s2 * sc_Kmax);
            *sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
            *sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);
        }
        //ok_str = ok?"true":"false";
        //std::cout << "primitives(): RSL ("<< ok_str <<"): " << std::to_string(*sc_s1) << ", " << std::to_string(*sc_s2) << ", " << std::to_string(*sc_s3) << ": " << std::to_string(*sc_s1+ *sc_s2 + *sc_s3) << std::endl;
        break;

        // RLR
    case 4:
        C = cos(sc_th0) - cos(sc_thf);
        S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
        temp1 = atan2(C, S);
        temp2 = 0.125 * (6 - 4 * pow(sc_Kmax, 2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
        if (abs(temp2) > 1) {
            ok = false;
            *sc_s1 = 0;
            *sc_s2 = 0;
            *sc_s3 = 0;
        }
        else {
            *sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
            *sc_s1 = invK * mod2pi(sc_th0 - temp1 + 0.5 * *sc_s2 * sc_Kmax);
            *sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (*sc_s2 - *sc_s1));
            ok = true;
        }
        //ok_str = ok?"true":"false";
        //std::cout << "primitives(): RLR ("<< ok_str <<"): " << std::to_string(*sc_s1) << ", " << std::to_string(*sc_s2) << ", " << std::to_string(*sc_s3) << ": " << std::to_string(*sc_s1+ *sc_s2 + *sc_s3) << std::endl;
        break;
        // LRL
    case 5:
        C = cos(sc_thf) - cos(sc_th0);
        S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
        temp1 = atan2(C, S);
        temp2 = 0.125 * (6 - 4 * pow(sc_Kmax, 2) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
        if (abs(temp2) > 1) {
            ok = false;
            *sc_s1 = 0;
            *sc_s2 = 0;
            *sc_s3 = 0;
        }
        else {
            *sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
            *sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * (*sc_s2) * sc_Kmax);
            *sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (*sc_s2 - *sc_s1));
            ok = true;
        }
        //ok_str = ok?"true":"false";
        //std::cout << "primitives(): LRL ("<< ok_str <<"): " << std::to_string(*sc_s1) << ", " << std::to_string(*sc_s2) << ", " << std::to_string(*sc_s3) << ": " << std::to_string(*sc_s1+ *sc_s2 + *sc_s3) << std::endl << std::endl;
        break;
    default:
        ok = false;
        *sc_s1 = 0;
        *sc_s2 = 0;
        *sc_s3 = 0;
        break;
    };

    return ok;
}


bool Dubins::checkIntersection(DubinsStructure curve){

  bool toReturn = false;
  if(curve.a1.L!=0) toReturn = toReturn || intersection_seg_arc(curve.a1.x0+(curve.a1.x0-curve.a1.xf)/2, curve.a1.y0+(curve.a1.y0-curve.a1.yf)/2, curve.a1.x0-curve.a1.xf);
  if(curve.a2.L!=0) toReturn = toReturn || intersection_seg_seg(curve.a2.x0, curve.a2.y0, curve.a2.xf, curve.a2.yf);
  if(curve.a3.L!=0) toReturn = toReturn || intersection_seg_arc(curve.a3.x0+(curve.a3.x0-curve.a3.xf)/2, curve.a3.y0+(curve.a3.y0-curve.a3.yf)/2, curve.a3.x0-curve.a3.xf);

  return toReturn;
}

bool Dubins::intersection_seg_seg(float x3, float y3, float x4, float y4){
  bool toReturn = false;

  for(obstacles_msgs::msg::ObstacleMsg obstacle : obstacles->obstacles){
    for(uint32_t i = 0; i<obstacle.polygon.points.size(); i++){
      double x1 = obstacle.polygon.points[i].x;
      double y1 = obstacle.polygon.points[i].y;
      double x2 = (i<obstacle.polygon.points.size()-1) ? obstacle.polygon.points[i+1].x : obstacle.polygon.points[0].x;
      double y2 = (i<obstacle.polygon.points.size()-1) ? obstacle.polygon.points[i+1].x : obstacle.polygon.points[0].y;

      double det = (x4-x3)*(y1-y2) - (x1-x2)*(y4-y3);

      double t = ((y3-y4)*(x1-x3)+(x4-x3)*(y1-y3))/det;
      double u = ((y1-y2)*(x1-x3)+(x2-x1)*(y1-y3))/det;

      toReturn = toReturn || (0<=t && t<=1) || (0<=u && u<=1);
    }
  }

  return toReturn;
}

bool Dubins::intersection_seg_arc(float xc, float yc, double r){
  bool toReturn = false;

  for(obstacles_msgs::msg::ObstacleMsg obstacle : obstacles->obstacles){
    for(uint32_t i = 0; i<obstacle.polygon.points.size(); i++){
      double x1 = obstacle.polygon.points[i].x;
      double y1 = obstacle.polygon.points[i].y;
      double x2 = (i<obstacle.polygon.points.size()-1) ? obstacle.polygon.points[i+1].x : obstacle.polygon.points[0].x;
      double y2 = (i<obstacle.polygon.points.size()-1) ? obstacle.polygon.points[i+1].x : obstacle.polygon.points[0].y;

      double a = std::pow((x2-x1),2) + std::pow((y2-y1), 2);
      double b = (x2-x1)*(x1-xc) + (y2-y1)*(y1-yc);
      double c = std::pow((x1-xc),2)+std::pow((y1-yc),2) - std::pow(r,2);

      if((b*b - a*c)>=0){
        double t1 = (-b+std::sqrt(b*b - a*c))/a;
        double t2 = (-b-std::sqrt(b*b - a*c))/a;

        toReturn = toReturn || (0<=t1 && t1<=1) || (0<=t2 && t2<=1);
      }
    }
  }

  return toReturn;
}

double Dubins::sinc(double x) {
    if (x == 0) return 1;
    else return sin(x) / x;
}

DubinsArc Dubins::circline( DubinsArc c) {
    c.xf = c.x0 + c.L * sinc(c.k * c.L / 2.0) * cos(c.th0 + c.k * c.L / 2);
    c.yf = c.y0 + c.L * sinc(c.k * c.L / 2.0) * sin(c.th0 + c.k * c.L / 2);
    c.thf = mod2pi(c.th0 + c.k * c.L);
    return c;
}

DubinsArc Dubins::dubinsarc(float x0, float y0, float th0, float  k, float L) {
    DubinsArc c = DubinsArc();
    c.x0 = x0;
    c.y0 = y0;
    c.th0 = th0;
    c.k = k;
    c.L = L;

    c.xf = x0 + L * sinc(k * L / 2.0) * cos(th0 + k * L / 2);
    c.yf = y0 + L * sinc(k * L / 2.0) * sin(th0 + k * L / 2);
    c.thf = mod2pi(th0 + k * L);

    return c;
}

DubinsStructure Dubins::dubinscurve(float x0, float y0, float th0, float s1, float s2, float s3, float k0, float k1, float k2) {
    DubinsStructure d = DubinsStructure();
    d.a1 = dubinsarc(x0, y0, th0, k0, s1);
    d.a2 = dubinsarc(d.a1.xf, d.a1.yf, d.a1.thf, k1, s2);
    d.a3 = dubinsarc(d.a2.xf, d.a2.yf, d.a2.thf, k2, s3);
    d.L = d.a1.L + d.a2.L + d.a3.L;
    return d;
}

void Dubins::addSinglePoints(std::vector<geometry_msgs::msg::PoseStamped>& pathList, rclcpp::Time stamp, float x, float y, float th) {

  geometry_msgs::msg::PoseStamped temp;
  temp.header.stamp = stamp;
  temp.header.frame_id = "";

  temp.pose.position.x = x;
  temp.pose.position.y = y;
  temp.pose.position.z = 0 ;

  temp.pose.orientation.x = 0;
  temp.pose.orientation.y = 0;
  temp.pose.orientation.z = 0;
  temp.pose.orientation.w = th;

  pathList.push_back(temp);
}

void Dubins::addArcWithIntrapoints(std::vector<geometry_msgs::msg::PoseStamped>& pathList, rclcpp::Time stamp, DubinsArc a, int num_intrapoints, bool addFirst) {
  int j = addFirst?0:1;
  for(; j<num_intrapoints; j++){
        geometry_msgs::msg::PoseStamped temp;
        float s_j = a.L/num_intrapoints * j;

        temp.header.stamp = stamp;
        temp.header.frame_id = "";

        temp.pose.position.x = a.x0 + s_j * sinc(a.k * s_j / 2.0) * cos(a.th0 + a.k * s_j / 2);
        temp.pose.position.y = a.y0 + s_j * sinc(a.k * s_j / 2.0) * sin(a.th0 + a.k * s_j / 2);
        temp.pose.position.z = 0 ;

        temp.pose.orientation.x = 0;
        temp.pose.orientation.y = 0;
        temp.pose.orientation.z = 0;
        temp.pose.orientation.w = mod2pi(a.th0 + a.k * s_j);
        pathList.push_back(temp);
  }
}

float Dubins::chooseEndingOrientation(float finalX, float finalY, float nextX, float nextY){
  return atan2((nextY-finalY), (nextX-finalX));
}
