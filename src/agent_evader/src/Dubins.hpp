#include <chrono>
#include <functional>
#include <math.h>
#include <memory>
#include <string>
#include <list>
#include <map>

#include "DubinsStructure.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std;

class Dubins {

public:

      DubinsStructure dubins_shortest_path(float x0, float y0, float th0, float xf, float yf, float thf, float Kmax);
      void dubins_full_path(float originalTh, rclcpp::Time stamp, list<geometry_msgs::msg::Point> pathToFollow, std::vector<geometry_msgs::msg::PoseStamped>& pathList);
      void addSinglePoints(std::vector<geometry_msgs::msg::PoseStamped>& pathList, rclcpp::Time stamp, float x, float y, float th);

private:

      int ksigns[6][3] = { {1,0,1},{-1,0,-1},{1,0,-1},{-1,0,1},{-1,1,-1},{1,-1,1} };
      float mod2pi(float ang);
      bool primitives(int type, float sc_th0, float sc_thf, float sc_Kmax, float* sc_s1, float* sc_s2, float* sc_s3);
      double sinc(double x);
      DubinsArc circline(DubinsArc c);
      DubinsArc dubinsarc(float x0, float y0, float th0, float  k, float L);
      DubinsStructure dubinscurve(float x0, float y0, float th0, float s1, float s2, float s3, float k0, float k1, float k2);
      void addArcWithIntrapoints(std::vector<geometry_msgs::msg::PoseStamped>& pathList, rclcpp::Time stamp, DubinsArc a, int num_intrapoints, bool addFirst);
      float chooseEndingOrientation(float finalX, float finalY, float nextX, float nextY);
};
