#include <list>
#include <memory>
#include <random>
#include <iostream>
#include <fstream>
#include <stdint.h>

#include "clipper_library/library_header.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "graph_msgs/msg/geometry_graph.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"

using std::placeholders::_1;
using namespace std;
using namespace geometry_msgs::msg;
using namespace clipper;

bool FLAG_LOAD_MOCK_GRAPH = true;

bool flagObstacles = false;
bool flagMapborders = false;
bool flagGates = false;
bool flagPositionShelfino1 = false;
bool flagPositionShelfino2 = false;
bool graph_initialized = false;
bool missingRoadmap = true;

graph_msgs::msg::GeometryGraph graph;

class RoadmapPublisher : public rclcpp::Node {
public:

  RoadmapPublisher(): Node("RoadmapPublisher"){
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    subscription1_ = this->create_subscription<geometry_msgs::msg::Polygon>(
        "map_borders", qos, std::bind(&RoadmapPublisher::topic_callback_map_borders, this, _1));
    subscription2_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "obstacles", qos, std::bind(&RoadmapPublisher::topic_callback_obstacles, this, _1));
    subscription3_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "gate_position", qos, std::bind(&RoadmapPublisher::topic_callback_gates, this, _1));
    subscription4_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "shelfino1/transform", qos, std::bind(&RoadmapPublisher::topic_callback_position_1, this, _1));
    subscription5_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "shelfino2/transform", qos, std::bind(&RoadmapPublisher::topic_callback_position_2, this, _1));

    publisher_ = this->create_publisher<graph_msgs::msg::GeometryGraph>("roadmap", 10);
  }

private:
  const static int NR_POINTS = 100;
  const static int K = 20;
  constexpr static float INFLATION_PARAMETER = 0.36; // cm
  constexpr static float SCALING_FACTOR = 100;

  geometry_msgs::msg::Polygon::SharedPtr map_borders;
  geometry_msgs::msg::PoseArray::SharedPtr gates;
  Point shelfino_position_1;
  Point shelfino_position_2;

  obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr obstacles;
  array<Point, NR_POINTS> pointList;
  float distance_matrix[NR_POINTS][NR_POINTS];
  int adjacency_matrix[NR_POINTS][NR_POINTS] = {0};

  void topic_callback_map_borders(const geometry_msgs::msg::Polygon::SharedPtr msg){
    if(flagMapborders){
      return;
    }
    else{
      flagMapborders = true;
      map_borders = msg;
      RCLCPP_INFO(this->get_logger(), "Map borders received!");
      for (auto ptr = (*map_borders).points.begin(); ptr < (*map_borders).points.end(); ptr++){
        RCLCPP_INFO(this->get_logger(), "Corner: (x: %f, y %f) ", ptr->x, ptr->y);
      }
    }
  }

  void topic_callback_gates(const geometry_msgs::msg::PoseArray::SharedPtr msg){
    if(flagGates){
      return;
    }
    else{
      flagGates = true;
      gates = msg;
      RCLCPP_INFO(this->get_logger(), "Position from gates retrieved!");
      for (uint32_t i = 0; i < gates->poses.size(); i++){
        RCLCPP_INFO(this->get_logger(), "Gate position: (x: %f, y %f) ", gates->poses[i].position.x, gates->poses[i].position.y);
      }
    }
  }

  void topic_callback_position_1(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    if(flagPositionShelfino1){
      return;
    }
    else{
      flagPositionShelfino1 = true;
      shelfino_position_1 = createPoint(msg->transform.translation.x,msg->transform.translation.y);
      RCLCPP_INFO(this->get_logger(), "Position from shelfino 1 retrieved: (%f, %f)", shelfino_position_1.x, shelfino_position_1.y);
    }
  }

  void topic_callback_position_2(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    if(flagPositionShelfino2){
      return;
    }
    else{
      flagPositionShelfino2 = true;
      shelfino_position_2 = createPoint(msg->transform.translation.x,msg->transform.translation.y);
      RCLCPP_INFO(this->get_logger(), "Position from shelfino 2 retrieved: (%f, %f)", shelfino_position_2.x, shelfino_position_2.y);
    }
  }


  void topic_callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg){

    if(!flagObstacles){
      flagObstacles = true;
      RCLCPP_INFO(this->get_logger(), "Obstacles list received!");
    }


    if (missingRoadmap && flagObstacles && flagMapborders && flagGates && flagPositionShelfino1 && flagPositionShelfino2){
      roadMapMain(msg);
      missingRoadmap = false;
      graph_initialized = true;
    }
    else if(missingRoadmap){
        RCLCPP_INFO(this->get_logger(), "Waiting for obstacles, borders, gates, evader or persecutor");
    }

    if (graph_initialized == true) {
      RCLCPP_INFO(this->get_logger(), "Publishing roadmap graph!");
      publisher_->publish(graph);
    }

  }

  void sample_points(){

    // Find min/max x/y values, assuming the map is a rectangle
    vector<Point32> copyMapBorder = map_borders->points;
    std::sort(copyMapBorder.begin(), copyMapBorder.end(), [](const Point32 &f, const Point32 &s){
       return f.x < s.x;
    });
    std::sort(copyMapBorder.begin(),copyMapBorder.end(),[](const Point32 &f, const Point32 &s){
      return f.y < s.y;
    });

    float min_x_value = copyMapBorder.front().x;
    float max_x_value = copyMapBorder.back().x;
    float min_y_value = copyMapBorder.front().y;
    float max_y_value = copyMapBorder.back().y;

    RCLCPP_INFO(this->get_logger(), "Min x value: %f, Max x value: %f", min_x_value, max_x_value);
    RCLCPP_INFO(this->get_logger(), "Min y value: %f, Max y value: %f", min_y_value, max_y_value);

    // generate #NR_POINTS Points
    for (uint32_t i = 0; i < (NR_POINTS - gates->poses.size() - 2); ){
      // Generate random point within map borders
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
      RCLCPP_DEBUG(this->get_logger(), "Point ID: %d (Random x: %f, random y: %f)", i, randomPoint.x, randomPoint.y);

      // Make sure the point is in Cfree
      bool pointInObstacle = checkIfInObstacle(randomPoint);
      bool pointOutsideMap = checkIfOutsideMap(randomPoint);

      if (pointInObstacle == true || pointOutsideMap == true){
        RCLCPP_DEBUG(this->get_logger(), "Not in C-free. Did collide with an obstacle? %s. Is it outside the map? %s", pointInObstacle ? "true" : "false", pointOutsideMap ? "true" : "false");
      }
      else{
        pointList[i] = randomPoint;
        i++;
      }
    }

    // add gate to PointList
    for (uint32_t g = 0; g < gates->poses.size(); g++){
      Point gate_point = createPoint(gates->poses[g].position.x, gates->poses[g].position.y);
      pointList[NR_POINTS - 2 - (g + 1)] = gate_point;
    }

    //  add evader position to list
    pointList[NR_POINTS-2] = shelfino_position_1;

    // add pursuer position to list
    pointList[NR_POINTS-1] = shelfino_position_2;

  }

  geometry_msgs::msg::Point createPoint(float x, float y){
    geometry_msgs::msg::Point pnt;
    pnt.x = x;
    pnt.y = y;
    pnt.z = 0;

    return pnt;
  }

  geometry_msgs::msg::Point32 createPoint32(float x, float y){
    geometry_msgs::msg::Point32 pnt32;
    pnt32.x = x;
    pnt32.y = y;
    pnt32.z = 0;

    return pnt32;
  }

  geometry_msgs::msg::Point point32ToPoint(geometry_msgs::msg::Point32 pnt32){
    geometry_msgs::msg::Point pnt;
    pnt.x = pnt32.x;
    pnt.y = pnt32.y;
    pnt.z = 0;

    return pnt;
  }

  bool checkIfOutsideMap(Point randomPoint){
    int n = (*map_borders).points.size();
    bool pointIsOutsideMap = !(rayCastingAlgorithm((*map_borders).points, n, randomPoint));

    return pointIsOutsideMap;
  }

  bool checkIfInObstacle(Point randomPoint){

    bool collisionDetected = false;
    for (obstacles_msgs::msg::ObstacleMsg obstacle : ((*obstacles).obstacles)){

      // Case 1: Obstacle is a circle
      if (obstacle.radius != 0){
        Point center_point = point32ToPoint(obstacle.polygon.points[0]);
        float distance_p2p = sqrt(pow(abs(center_point.x - randomPoint.x), 2) + pow(abs(center_point.y - randomPoint.y), 2));
        collisionDetected = distance_p2p <= obstacle.radius;
      }

      // Case 2: Another polygone
      else if (obstacle.radius == 0){
        int n = obstacle.polygon.points.size();
        collisionDetected = rayCastingAlgorithm(obstacle.polygon.points, n, randomPoint);
      }

      if (collisionDetected == true) break;

    }

    return collisionDetected;
  }

  // copied from https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
  bool rayCastingAlgorithm(vector<Point32> polygon32, int n, Point p){

    vector<Point> polygon;
    for (Point32 point32 : polygon32){
      polygon.push_back(point32ToPoint(point32));
    }

    bool pointIsInside = checkInside(polygon, n, p);
    return pointIsInside;
  }

  void calculateDistanceMatrix(){
    for (uint32_t i = 0; i < pointList.size(); i++){
      Point point_i = pointList[i];

      for (uint32_t j = 0; j < pointList.size(); j++){
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

  void calculateAdjacencyMatrix(){

    // loop over each row of distance matrix to find each point's k nearest neighbors
    // int adjacency_matrix[pointList.size()][pointList.size()] = {0};
    for (uint32_t i = 0; i < pointList.size(); i++){

      array<int, K> nearest_neighbors;
      //float current_row[pointList.size()];
      float *current_row = new float[pointList.size()];

      for (uint32_t j = 0; j < pointList.size(); j++){
        current_row[j] = distance_matrix[i][j];
      }

      sort(current_row, current_row + pointList.size());
      list<int> used_points_indices;

      // Skipping the shortest first distance which is always 0
      for (int k = 1; k <= K; k++) {
        float current_distance = current_row[k];
        // find current_distance in distance_matrix

        int index_nearest_neighbor = i;
        for (uint32_t l = 0; l < pointList.size(); l++){

          // check if point l is already used as neighbor:
          bool found = (std::find(used_points_indices.begin(), used_points_indices.end(), l) != used_points_indices.end());
          if ((distance_matrix[i][l] == current_distance) && (found == false)){
            index_nearest_neighbor = l;
          }
        }
        used_points_indices.push_back(index_nearest_neighbor);
        nearest_neighbors[k - 1] = index_nearest_neighbor;
      }

      // TODO: in nearest_neighbors sind alle punkte die am nächsten zu pointList[i] sind.
      for (uint32_t m = 0; m < pointList.size(); m++){

        bool found = (std::find(nearest_neighbors.begin(), nearest_neighbors.end(), m) != nearest_neighbors.end());
        bool pathIsCollisionFree = checkPathIsCollisionFree(i, m);
        if ((found == true) && (i != m) && (pathIsCollisionFree)){
          adjacency_matrix[i][m] = 1;
          adjacency_matrix[m][i] = 1;
        }
      }
    }
  }

  void createGraph(){
    // add points to graph.nodes
    for (int i = 0; i < NR_POINTS; i++){
      graph.nodes.push_back(pointList[i]);
    }

    // create edges
    for (int i = 0; i < NR_POINTS; i++){
      graph_msgs::msg::Edges edge;
      graph.edges.push_back(edge);

      for (int j = 0; j < NR_POINTS; j++){
        if (adjacency_matrix[i][j] == 1){
          graph.edges[i].node_ids.push_back(j);
          graph.edges[i].weights.push_back(distance_matrix[i][j]);
        }
      }
    }
  }

  vector<string> split(string s, string delimiter) {
      size_t pos_start = 0, pos_end, delim_len = delimiter.length();
      string token;
      vector<string> res;

      while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
          token = s.substr (pos_start, pos_end - pos_start);
          pos_start = pos_end + delim_len;
          res.push_back (token);
      }

      res.push_back (s.substr (pos_start));
      return res;
  }

  void loadMockRoadmap(){
    string line;
    ifstream myfile ("Roadmap.txt");
    std::string delimiter = ";";
    std::string delimiter_point = " ";

    if (myfile.is_open()){
      for (int i = 0; i < NR_POINTS; i++){
        getline (myfile,line);
        vector<string> adjecent_line = split(line, delimiter);
        int index = 0;
        for (auto adjecent_cell : adjecent_line){
          adjacency_matrix[i][index] = stoi(adjecent_cell);
          index++;
        }
      }

      for (int row =0; row < NR_POINTS; row++){
        getline (myfile,line);
        vector<string> distance_line = split(line, delimiter);
        int index = 0;
        for (auto distance_cell : distance_line){
          distance_matrix[row][index] = stof(distance_cell);
          index++;
        }
      }

      uint32_t index_two = 0;
      while (getline (myfile,line) ){
        vector<string> points = split(line, delimiter_point);
        pointList[index_two].x = stof(points[0]);
        pointList[index_two].y = stof(points[1]);
        index_two++;
      }

      myfile.close();
    }

      cout<< "Initial point list: "<<pointList[0].x << ", "<<pointList[0].y<<endl;
      cout<< "Final point list: "<<pointList[99].x << ", "<<pointList[99].y<<endl;

  }

  void printMockRoadmap(){
    for(int row =0; row < NR_POINTS; row++){
      for(int col=0; col<NR_POINTS; col++){
        std::cout<< adjacency_matrix[row][col];
        if(col==(NR_POINTS-1))
          std::cout<<std::endl;
        else
          std::cout<<";";
      }
    }
    for(int row =0; row < NR_POINTS; row++){
      for(int col=0; col<NR_POINTS; col++){
        std::cout<< distance_matrix[row][col];
        if(col==(NR_POINTS-1))
          std::cout<<std::endl;
        else
          std::cout<<";";
      }
    }
    for(int i =0; i<NR_POINTS; i++){
      std::cout<<pointList[i].x<<" "<<pointList[i].y<<std::endl;
    }
  }

  struct Line{
    Point p1, p2;
  };

  bool onLine(Line l1, Point p){
    // Check whether p is on the line or not
    if (p.x <= max(l1.p1.x, l1.p2.x) && p.x <= min(l1.p1.x, l1.p2.x) && (p.y <= max(l1.p1.y, l1.p2.y) && p.y <= min(l1.p1.y, l1.p2.y)))
      return true;

    return false;
  }

  int direction(Point a, Point b, Point c){
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

  bool isIntersect(Line l1, Line l2){
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

  bool checkInside(vector<Point> poly, int n, Point p){

    // When polygon has less than 3 edge, it is not polygon
    if (n < 3){
      RCLCPP_WARN(this->get_logger(), "Something went wrong with the polygon conversion");
      return false;
    }

    // Create a point at infinity, y is same as point p
    Line exline = {p, createPoint(9999.0, p.y)};
    int count = 0;
    int i = 0;
    do{

      // Forming a Line from two consecutive points of
      // poly
      Line side = {poly[i], poly[(i + 1) % n]};
      if (isIntersect(side, exline)){

        // If side is intersects exline
        if (direction(side.p1, p, side.p2) == 0){
          RCLCPP_DEBUG(this->get_logger(), "Point p (%f, %f) is on a line ([%f,%f]--->[%f,%f])", p.x, p.y, side.p1.x, side.p1.y,side.p2.x, side.p2.y);
          return onLine(side, p);
        }

        count++;
      }
      i = (i + 1) % n;
    } while (i != 0);

    // When count is odd
    return count & 1;
  }

  bool checkPathIsCollisionFree(int k, int m){

    bool collisionFree = true;
    Line pathLine = {pointList[k], pointList[m]};
    for (obstacles_msgs::msg::ObstacleMsg obstacle : ((*obstacles).obstacles)){

      vector<Point> polygon;
      for (Point32 point32 : obstacle.polygon.points){
        polygon.push_back(point32ToPoint(point32));
      }

      int n = polygon.size();
      int i = 0;
      do {
        Line side = {polygon[i], polygon[(i + 1) % n]};
        if (isIntersect(side, pathLine)){
          collisionFree = false;
          break;
        }
        i = (i + 1) % n;

      } while (i != 0);
    }

    return collisionFree;
  }

  void inflatePolygon(double delta, Polygon *polygon){

    ClipperOffset co;
    co.Clear();
    vector<IntPoint> intPoints = scalePoints((*polygon).points);

    co.AddPath(intPoints, jtMiter, etClosedPolygon);
    Paths result;
    co.Execute(result, delta);

    (*polygon).points = descalePoints(result.front());

    if (result.size() != 1){
      RCLCPP_WARN(this->get_logger(), "Something went wrong when inflating the Polygon");
    }
    else{
      RCLCPP_DEBUG(this->get_logger(), "Polygon has been inflated");
      for (uint32_t i = 0; i < (*polygon).points.size(); i++){
        RCLCPP_DEBUG(this->get_logger(), "Vertix %u: (%f, %f)", i, (*polygon).points[i].x, (*polygon).points[i].y);
      }
    }
  }

  vector<IntPoint> scalePoints(vector<Point32> points32){

    vector<IntPoint> scaledIntPoints;

    for (uint32_t i = 0; i < points32.size(); i++){
      IntPoint *intPoint = new IntPoint((points32[i].x * SCALING_FACTOR), (points32[i].y * SCALING_FACTOR));
      scaledIntPoints.push_back(*intPoint);
    }

    return scaledIntPoints;
  }

  vector<Point32> descalePoints(vector<IntPoint> scaledPoints){
    vector<Point32> descaledPoints32;
    for (uint32_t i = 0; i < scaledPoints.size(); i++){
      Point32 point32;
      point32.x = ((scaledPoints[i].X) / SCALING_FACTOR);
      point32.y = ((scaledPoints[i].Y) / SCALING_FACTOR);
      point32.z = 0;
      descaledPoints32.push_back(point32);
    }

    return descaledPoints32;
  }

  void roadMapMain(obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg){

    obstacles = msg;

    RCLCPP_INFO(this->get_logger(), "Inflating the map borders");
    inflatePolygon((-1 * (INFLATION_PARAMETER * SCALING_FACTOR)), map_borders.get());

    RCLCPP_INFO(this->get_logger(), "Inflating the map obstacles");
    for (uint32_t o = 0; o < (*obstacles).obstacles.size(); o++){
      inflatePolygon((INFLATION_PARAMETER * SCALING_FACTOR), &(*obstacles).obstacles[o].polygon);
    }

    if(FLAG_LOAD_MOCK_GRAPH){
      RCLCPP_INFO(this->get_logger(), "Retrieving a old roadmap");
      loadMockRoadmap();
    }
    else{
      RCLCPP_INFO(this->get_logger(), "Creating the roadmap with the real information");
      sample_points();
      calculateDistanceMatrix();
      calculateAdjacencyMatrix();
    }
    createGraph();
    //printMockRoadmap();

  }

  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription1_;
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription2_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription3_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription4_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription5_;
  rclcpp::Publisher<graph_msgs::msg::GeometryGraph>::SharedPtr publisher_;
};

int main(int argc, char *argv[]){

  rclcpp::init(argc, argv);
  cout << "---> Roadmap Publisher started..." << '\n';
  rclcpp::spin(std::make_shared<RoadmapPublisher>());

  if (flagObstacles == true && flagMapborders == true){
    cout << "... stopping Roadmap Publisher" << '\n';
  }

  rclcpp::shutdown();
  return 0;
}
