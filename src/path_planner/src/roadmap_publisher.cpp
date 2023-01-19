#include <memory>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "std_msgs/msg/header.hpp"
#include "graph_msgs/msg/geometry_graph.hpp"
#include "clipper_library/library_header.h"

#include <list>

using std::placeholders::_1;
using namespace std;
using namespace geometry_msgs::msg;
using namespace clipper;
bool flagObstacles = false;
bool flagMapborders = true;
bool flagGates = true;

int executeOnce = 0;
graph_msgs::msg::GeometryGraph graph;

class MinimalSubscriber : public rclcpp::Node
{
public:
  // geometry_msgs::msg::Polygon map_borders;

  MinimalSubscriber()
      : Node("roadmapPublisher")
  {

    /*subscription1_ = this->create_subscription<geometry_msgs::msg::Polygon>(
        "map_borders", 10, std::bind(&MinimalSubscriber::topic_callback_map_borders, this, _1));*/
    subscription2_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "obstacles", 10, std::bind(&MinimalSubscriber::topic_callback_obstacles, this, _1));
    /*subscription3_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "gate_position", 10, std::bind(&MinimalSubscriber::topic_callback_gates, this, _1));*/

    publisher_ = this->create_publisher<graph_msgs::msg::GeometryGraph>("roadmap", 10);
  }

private:
  const static int NR_POINTS = 100;
  const static int K = 20;
  constexpr static float INFLATION_PARAMETER = 0.36; // cm
  constexpr static float SCALING_FACTOR = 100;

  geometry_msgs::msg::Polygon *map_borders;
  geometry_msgs::msg::Point *gate;

  obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr obstacles;
  array<Point, NR_POINTS> pointList;
  float distance_matrix[NR_POINTS][NR_POINTS];
  int adjacency_matrix[NR_POINTS][NR_POINTS] = {0};

  // only needed when /map_borders topic will be available.
  /*void topic_callback_map_borders(const geometry_msgs::msg::Polygon::SharedPtr msg)
  {

    RCLCPP_INFO(this->get_logger(), "I heard something else: ", msg);

    map_borders = msg->polygon;
    for (auto ptr = map_borders.points.begin(); ptr < map_borders.points.end(); ptr++)
    {
      cout << "ptr->x" << ptr->x<< '\n';
      cout << "ptr->y" << ptr->y<< '\n';
    }
  }
  void topic_callback_gates(const geometry_msgs::msg::Pose::SharedPtr msg)
  {

    RCLCPP_INFO(this->get_logger(), "I heard something about the gate: ", msg);

    gate = msg->position;
    flagGates = true;
    cout << "gate position: " << *gate.x << " / " << *gate.y << '\n';
  }*/

  void topic_callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: ", msg);
    flagObstacles = true;
    executeOnce += 1;
    if (flagObstacles == true && flagMapborders == true && flagGates == true && executeOnce == 1)
    {
      roadMapMain(msg);
    }
    logResult();
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'");
    cout << "Iteration: " << executeOnce << '\n';
    publisher_->publish(graph);
  }
  void sample_points()
  {

    // find min/max x/y values, assuming the map is a rectangle without rotation
    /*(map_borders->points).sort([](const Point &f, const Point &s)
                             { return f.x < s.x; });*/

    vector<Point32> copyMapBorder = map_borders->points;
    std::sort(
        copyMapBorder.begin(),
        copyMapBorder.end(),
        [](const Point32 &f, const Point32 &s)
        { return f.x < s.x; });

    float min_x_value = copyMapBorder.front().x;
    float max_x_value = copyMapBorder.back().x;

    cout << "MIN X VALUE: " << min_x_value << " / MAX X VALUE: " << max_x_value << '\n';
    /*(map_borders->points).sort([](const Point &f, const Point &s)
                               { return f.y < s.y; });*/

    std::sort(
        copyMapBorder.begin(),
        copyMapBorder.end(),
        [](const Point32 &f, const Point32 &s)
        { return f.y < s.y; });
    float min_y_value = copyMapBorder.front().y;
    float max_y_value = copyMapBorder.back().y;
    cout << "MIN Y VALUE: " << min_y_value << " / MAX Y VALUE: " << max_y_value << '\n';

    // generate #NR_POINTS Points
    for (int i = 0; i < (NR_POINTS - 1); i++)
    {
      // generate random point within map borders
      const int range_from_x = min_x_value;
      const int range_to_x = max_x_value;
      std::random_device rand_dev_x;
      std::mt19937 generator_x(rand_dev_x());
      std::uniform_real_distribution<float> distr_x(range_from_x, range_to_x);
      float random_x = distr_x(generator_x);

      const int range_from_y = min_y_value;
      const int range_to_y = max_y_value;
      std::random_device rand_dev_y;
      std::mt19937 generator_y(rand_dev_y());
      std::uniform_real_distribution<float> distr_y(range_from_y, range_to_y);
      float random_y = distr_y(generator_y);

      Point randomPoint = createPoint(random_x, random_y);
      std::cout << "Point ID: " << i << '\n';
      std::cout << "random_x: " << randomPoint.x << '\n';
      std::cout << "random_y: " << randomPoint.y << '\n';

      // make sure the point is in Cfree

      bool pointInObstacle = checkIfInObstacle(randomPoint);
      bool pointOutsideMap = checkIfOutsideMap(randomPoint);
      if (pointOutsideMap == true)
      {
        cout << "point is outside map, not ok" << '\n';
      }
      if (pointInObstacle == true)
      {
        cout << "point is in obstacle, not ok" << '\n';
      }

      if (pointInObstacle == true || pointOutsideMap == true) // || pointOutsideMap == true
      {
        i--;
        cout << "not in c free." << '\n';
      }
      else
      {
        pointList[i] = randomPoint;
      }
    }
    // add gate to PointList
    pointList[NR_POINTS - 1] = *gate;
    cout << "GATE: " << pointList[NR_POINTS].x << " / " << pointList[NR_POINTS].y << '\n';
    // add evader position to list
    /*pointList[NR_POINTS] = *evader_pose;

    // add pursuer position to list
    pointList[NR_POINTS] = *pursuer_pose;
    */
  }

  geometry_msgs::msg::Point createPoint(float x, float y)
  {
    geometry_msgs::msg::Point pnt;
    pnt.x = x;
    pnt.y = y;
    pnt.z = 0;

    return pnt;
  }
  geometry_msgs::msg::Point32 createPoint32(float x, float y)
  {
    geometry_msgs::msg::Point32 pnt32;
    pnt32.x = x;
    pnt32.y = y;
    pnt32.z = 0;

    return pnt32;
  }
  geometry_msgs::msg::Point point32ToPoint(geometry_msgs::msg::Point32 pnt32)
  {
    geometry_msgs::msg::Point pnt;
    pnt.x = pnt32.x;
    pnt.y = pnt32.y;
    pnt.z = 0;

    return pnt;
  }

  bool checkIfOutsideMap(Point randomPoint)
  {
    int n = (*map_borders).points.size();
    bool pointIsOutsideMap = !(rayCastingAlgorithm((*map_borders).points, n, randomPoint));

    return pointIsOutsideMap;
  }

  bool checkIfInObstacle(Point randomPoint)
  {

    bool collisionDetected = false;
    // obstacles, array of polygons.
    for (obstacles_msgs::msg::ObstacleMsg obstacle : ((*obstacles).obstacles))
    {
      if (obstacle.radius != 0)
      {
        // check if point is in circle
        // cout << "Obstacle is circle" << '\n';
        Point center_point = point32ToPoint(obstacle.polygon.points[0]);
        float distance_p2p = sqrt(pow(abs(center_point.x - randomPoint.x), 2) + pow(abs(center_point.y - randomPoint.y), 2));
        if (distance_p2p <= obstacle.radius)
        {
          // cout << "Point is inside circle." << '\n';
          collisionDetected = true;
        }
        continue;
      }
      else if (obstacle.radius == 0)
      {

        // check if point is in polygon
        // cout << "Obstacle is polygon" << '\n';
        int n = obstacle.polygon.points.size();

        collisionDetected = rayCastingAlgorithm(obstacle.polygon.points, n, randomPoint);
      }
      if (collisionDetected == true)
      {
        // cout << " Collision detected" << '\n';
        break;
      }
    }
    return collisionDetected;
  }
  bool rayCastingAlgorithm(vector<Point32> polygon32, int n, Point p) // copied from https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
  {

    vector<Point> polygon;
    for (Point32 point32 : polygon32)
    {
      polygon.push_back(point32ToPoint(point32));
    }

    bool pointIsInside = checkInside(polygon, n, p);
    return pointIsInside;
  }

  void calculateDistanceMatrix()
  {
    for (int i = 0; i < pointList.size(); i++)
    {
      Point point_i = pointList[i];
      for (int j = 0; j < pointList.size(); j++)
      {
        Point point_j = pointList[j];

        float delta_x = abs(point_i.x - point_j.x);
        float delta_y = abs(point_i.y - point_j.y);
        float delta_x_sq = pow(delta_x, 2);
        float delta_y_sq = pow(delta_y, 2);
        float distance = sqrt(delta_x_sq + delta_y_sq);
        distance_matrix[i][j] = distance;
      }
    }
  }

  void calculateAdjacencyMatrix()
  {
    for (int test = 0; test < pointList.size(); test++)
    {
      Point test_point = pointList[test];
      std::cout << '\n';
      std::cout << "Point " << test << ": " << test_point.x << " / " << test_point.y << '\n';
      std::cout << "Distances to other points: ";
      for (int j = 0; j < pointList.size(); j++)
      {
        std::cout << distance_matrix[test][j] << " / ";
      }
      std::cout << '\n';
    }

    // loop over each row of distance matrix to find each point's k nearest neighbors
    // int adjacency_matrix[pointList.size()][pointList.size()] = {0};
    for (int i = 0; i < pointList.size(); i++)
    {
      array<int, K> nearest_neighbors;
      float current_row[pointList.size()];
      for (int j = 0; j < pointList.size(); j++)
      {
        current_row[j] = distance_matrix[i][j];
      }

      sort(current_row, current_row + pointList.size());
      list<int> used_points_indices;
      for (int k = 1; k <= K; k++) // skipping the shortest first distance which is always 0
      {
        float current_distance = current_row[k];
        // find current_distance in distance_matrix

        // cout << "used_points_indices: ";
        int index_nearest_neighbor = i;
        for (int l = 0; l < pointList.size(); l++)
        {

          // check if point l is already used as neighbor:
          bool found = (std::find(used_points_indices.begin(), used_points_indices.end(), l) != used_points_indices.end());
          if ((distance_matrix[i][l] == current_distance) && (found == false))
          {
            // cout << "FOUND THE NEIGHBOR: " << l << ", K = " << k << '\n';
            index_nearest_neighbor = l;
          }
        }
        used_points_indices.push_back(index_nearest_neighbor);
        nearest_neighbors[k - 1] = index_nearest_neighbor;
      }

      // test: print nearest points.
      // cout << "Nearest Neighbours Point " << i << ": " << '\n';
      for (int test3 = 0; test3 < nearest_neighbors.size(); test3++)
      {
        // cout << nearest_neighbors[test3] << '\n';
      }
      cout << '\n';

      // TADA: in nearest_neighbors sind alle punkte die am nächsten zu pointList[i] sind.
      for (int m = 0; m < pointList.size(); m++)
      {

        bool found = (std::find(nearest_neighbors.begin(), nearest_neighbors.end(), m) != nearest_neighbors.end());
        bool pathIsCollisionFree = checkPathIsCollisionFree(i, m);
        if ((found == true) && (i != m) && (pathIsCollisionFree))
        {
          adjacency_matrix[i][m] = 1;
          adjacency_matrix[m][i] = 1;
        }
      }
    }
  }

  void createGraph()
  {
    // add points to graph.nodes
    for (int i = 0; i < NR_POINTS; i++)
    {
      graph.nodes.push_back(pointList[i]);
    }

    // create edges
    for (int i = 0; i < NR_POINTS; i++)
    {
      graph_msgs::msg::Edges edge;
      graph.edges.push_back(edge);
      for (int j = 0; j < NR_POINTS; j++)
      {
        if (adjacency_matrix[i][j] == 1)
        {
          graph.edges[i].node_ids.push_back(j);
        }
      }
    }
  }

  void logResult()
  {
    // print adjacency matrix
    cout << "A: " << '\n';
    cout << "[";
    for (int i = 0; i < NR_POINTS; i++)
    {
      for (int j = 0; j < NR_POINTS; j++)
      {
        cout << adjacency_matrix[i][j] << " ";
      }
      cout << ";";
    }
    cout << "]";
    cout << '\n';
    // print x
    cout << "X: " << '\n';
    cout << "[";
    for (int i = 0; i < pointList.size(); i++)
    {
      cout << pointList[i].x << " ";
    }
    cout << "]";
    cout << '\n';

    // print y
    cout << "Y: " << '\n';
    cout << "[";
    for (int i = 0; i < pointList.size(); i++)
    {
      cout << pointList[i].y << " ";
    }
    cout << "]";
    cout << '\n';
  }

  /* (copied)*/
  struct line
  {
    Point p1, p2;
  };

  bool onLine(line l1, Point p)
  {
    // Check whether p is on the line or not
    if (p.x <= max(l1.p1.x, l1.p2.x) && p.x <= min(l1.p1.x, l1.p2.x) && (p.y <= max(l1.p1.y, l1.p2.y) && p.y <= min(l1.p1.y, l1.p2.y)))
      return true;

    return false;
  }

  int direction(Point a, Point b, Point c)
  {
    double val = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);

    if (val == 0)

      // Colinear
      return 0;

    else if (val < 0)

      // Anti-clockwise direction
      return 2;

    // Clockwise direction
    return 1;
  }

  bool isIntersect(line l1, line l2)
  {
    // Four direction for two lines and points of other line
    int dir1 = direction(l1.p1, l1.p2, l2.p1);
    int dir2 = direction(l1.p1, l1.p2, l2.p2);
    int dir3 = direction(l2.p1, l2.p2, l1.p1);
    int dir4 = direction(l2.p1, l2.p2, l1.p2);

    // When intersecting
    if (dir1 != dir2 && dir3 != dir4)
      return true;

    // When p2 of line2 are on the line1
    if (dir1 == 0 && onLine(l1, l2.p1))
      return true;

    // When p1 of line2 are on the line1
    if (dir2 == 0 && onLine(l1, l2.p2))
      return true;

    // When p2 of line1 are on the line2
    if (dir3 == 0 && onLine(l2, l1.p1))
      return true;

    // When p1 of line1 are on the line2
    if (dir4 == 0 && onLine(l2, l1.p2))
      return true;

    return false;
  }

  bool checkInside(vector<Point> poly, int n, Point p)
  {

    // When polygon has less than 3 edge, it is not polygon
    if (n < 3)
    {

      cout << "SOMETHING WENT WRONG WITH POLYGON CONVERSION" << '\n';
      return false;
    }

    // Create a point at infinity, y is same as point p
    line exline = {p, createPoint(9999.0, p.y)};
    int count = 0;
    int i = 0;
    do
    {

      // Forming a line from two consecutive points of
      // poly
      line side = {poly[i], poly[(i + 1) % n]};
      if (isIntersect(side, exline))
      {

        // If side is intersects exline
        if (direction(side.p1, p, side.p2) == 0)
        {
          cout << "P " << p.x << " / " << p.y << " is on Line:";
          cout << side.p1.x << " / " << side.p1.y << " -> " << side.p2.x << " / " << side.p2.y << '\n';
          return onLine(side, p);
        }

        count++;
      }
      i = (i + 1) % n;
    } while (i != 0);

    // When count is odd
    return count & 1;
  }

  bool checkPathIsCollisionFree(int k, int m)
  {

    bool collisionFree = true;
    line pathLine = {pointList[k], pointList[m]};
    for (obstacles_msgs::msg::ObstacleMsg obstacle : ((*obstacles).obstacles))
    {
      vector<Point> polygon;
      for (Point32 point32 : obstacle.polygon.points)
      {
        polygon.push_back(point32ToPoint(point32));
      }
      int n = polygon.size();
      // cout << "CHECKPATH COLLISION FREE" << '\n';
      int i = 0;
      do
      {
        line side = {polygon[i], polygon[(i + 1) % n]};
        if (isIntersect(side, pathLine))
        {
          // cout << " IS INTERSECT!!" << '\n';
          collisionFree = false;
          break;
        }
        i = (i + 1) % n;
      } while (i != 0);
    }

    return collisionFree;
  }

  void inflatePolygon(double delta, Polygon *polygon)
  {
    ClipperOffset co;
    co.Clear();
    vector<IntPoint> intPoints = scalePoints((*polygon).points);

    co.AddPath(intPoints, jtMiter, etClosedPolygon);
    Paths result;
    co.Execute(result, delta);

    (*polygon).points = descalePoints(result.front());

    if (result.size() != 1)
    {
      cout << "Something went wrong when inflating the Polygon" << '\n';
    }
    else
    {
      cout << "Inflation should have worked: " << '\n';
      for (int i = 0; i < (*polygon).points.size(); i++)
      {
        cout << "Point " << i << ": X ";
        cout << (*polygon).points[i].x;
        cout << " / Y: ";
        cout << (*polygon).points[i].y << '\n';
      }
    }
  }

  vector<IntPoint> scalePoints(vector<Point32> points32)
  {

    vector<IntPoint> scaledIntPoints;

    for (int i = 0; i < points32.size(); i++)
    {
      IntPoint *intPoint = new IntPoint((points32[i].x * SCALING_FACTOR), (points32[i].y * SCALING_FACTOR));
      scaledIntPoints.push_back(*intPoint);
    }

    return scaledIntPoints;
  }

  vector<Point32> descalePoints(vector<IntPoint> scaledPoints)
  {
    vector<Point32> descaledPoints32;
    for (int i = 0; i < scaledPoints.size(); i++)
    {
      Point32 point32;
      point32.x = ((scaledPoints[i].X) / SCALING_FACTOR);
      point32.y = ((scaledPoints[i].Y) / SCALING_FACTOR);
      point32.z = 0;
      descaledPoints32.push_back(point32);
    }

    return descaledPoints32;
  }

  void roadMapMain(obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
  {
    for (int i = 0; i < (*msg).obstacles.size(); i++)
    {
      cout << "OBSTACLE " << i << '\n';
      for (int j = 0; j < (*msg).obstacles[i].polygon.points.size(); j++)
      {
        cout << "Point " << j << ": X ";
        cout << (*msg).obstacles[i].polygon.points[j].x;
        cout << " / Y: ";
        cout << (*msg).obstacles[i].polygon.points[j].y << '\n';
      }
    }

    obstacles = msg;
    Point32 p1 = createPoint32(0, 0);
    Point32 p2 = createPoint32(10, 0);
    Point32 p3 = createPoint32(10, 10);
    Point32 p4 = createPoint32(0, 10);
    Polygon poly;
    poly.points.push_back(p1);
    poly.points.push_back(p2);
    poly.points.push_back(p3);
    poly.points.push_back(p4);
    map_borders = &poly;

    Point pointGate = createPoint(0, 2);
    gate = &pointGate;
    cout << "Poiint Gate: " << (*gate).x << " / " << (*gate).y << '\n';

    cout << '\n';
    cout << "INFLATE MAP BORDERS" << '\n';
    inflatePolygon((-1 * (INFLATION_PARAMETER * SCALING_FACTOR)), map_borders);
    cout << '\n';
    cout << "INFLATE OBSTACLES" << '\n';
    for (int o = 0; o < (*obstacles).obstacles.size(); o++)
    {
      inflatePolygon((INFLATION_PARAMETER * SCALING_FACTOR), &(*obstacles).obstacles[o].polygon);
    }

    sample_points();
    calculateDistanceMatrix();
    calculateAdjacencyMatrix();
    createGraph();
    // logResult();
  }

  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription1_;
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription2_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription3_;
  rclcpp::Publisher<graph_msgs::msg::GeometryGraph>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  if (flagObstacles == true && flagMapborders == true)
  {
    cout << "HOLALLALALLALA" << '\n';
  }

  rclcpp::shutdown();
  return 0;
}