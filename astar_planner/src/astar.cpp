#include "astar.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <boost/thread.hpp>

#include <nav_msgs/Path.h>
#include <ros/package.h>

using namespace std;

ASTAR::ASTAR( ros::NodeHandle & nh ) {
  nh_ = &nh;
  waypoints_done_ = false;
  initialised_ = false;
  nh_->param<double>( "map_width", map_width_, 6.44 );
  nh_->param<double>( "map_height", map_height_, 3.33 );
  nh_->param<double>( "map_resolution", map_resolution_, 0.01 );
  nh_->param<double>( "grid_resolution", grid_resolution_, 0.06 );
  nh_->param<double>( "startx", start_[0], 0.00 );
  nh_->param<double>( "starty", start_[1], 1.50 );
  nh_->param<double>( "goalx", goal_[0], 2.50 );
  nh_->param<double>( "goaly", goal_[1], 1.50 );

  cout << "setting parameters ... done!\n";
  map_img_ = cv::imread( ros::package::getPath("astar_planner")+"/data/map.png", CV_LOAD_IMAGE_GRAYSCALE);
  plan_pub_ = nh.advertise<nav_msgs::Path>("plan", 1);

  boost::thread thread = boost::thread( boost::bind(&ASTAR::path_search, this) );
}


// data type convert functions
double ASTAR::grid2meterX(int x) {
  double nx = x*grid_resolution_-map_width_/2+grid_resolution_/2;
  return nx;
}

double ASTAR::grid2meterY(int y) {
  double ny = map_height_/2-y*grid_resolution_-grid_resolution_/2;
  return ny;
}

int ASTAR::meterX2grid(double x) {
  int gx = round((x+map_width_/2)/grid_resolution_);
  if ( gx > grid_width_-1 )
    gx = grid_width_-1;
  if ( gx < 0 )
    gx = 0;
  return gx;
}

int ASTAR::meterY2grid(double y) {
  int gy = round((map_height_/2-y)/grid_resolution_);
  if ( gy > grid_height_-1 )
    gy = grid_height_-1;
  if ( gy < 0 )
    gy = 0;
  return gy;
}

// print functions
void ASTAR::print_list(vector<Node> nodelist) {
  for ( int i = 0; i < (int)nodelist.size(); ++ i ) {
    Node & n = nodelist[i];
    cout << "idx [" << n.x << ", " << n.y << "] with cost = " << n.cost << endl;
  }
}

void ASTAR::print_grid_props(const char *c) {
    if ( strcmp(c, "c") == 0 )
        ROS_INFO("\n\n---------------Grid_property: closed status---------------");
    else if ( strcmp(c, "h") == 0 )
        ROS_INFO("\n\n---------------Grid_property: heuristic cost---------------");
    else if ( strcmp(c, "e") == 0 )
        ROS_INFO("\n\n---------------Grid_property: expansion level---------------");
    else if ( strcmp(c, "o") == 0 )
        ROS_INFO("\n\n---------------Grid_property: occupied status---------------");
    
  for ( int i = 0; i < grid_height_; ++ i ) {
    for ( int j = 0; j < grid_width_; ++ j ) {
      if ( strcmp(c, "c") == 0 )
        cout << gridmap_[i][j].closed << " ";
      else if ( strcmp(c, "h") == 0 )
        cout << gridmap_[i][j].heuristic << " ";
      else if ( strcmp(c, "e") == 0 )
        cout << gridmap_[i][j].expand << " ";
      else if ( strcmp(c, "o") == 0 )
        cout << gridmap_[i][j].occupied << " ";
    }
    cout << endl;
  }
  cout << endl;
}

void ASTAR::print_waypoints(vector<Waypoint> waypoints) {
  cout << "Waypoints:\n";
  for ( int i = 0; i < (int)waypoints.size(); ++ i ) {
    Waypoint & w = waypoints[i];
    cout << "[" << w.x << ", " << w.y << "]\n";
  }
}

bool ASTAR::inRange(int ytest, int xtest)
{
    if ((ytest >= 0 && ytest < gridmap_.size()) && (xtest>=0 && xtest < gridmap_[ytest].size()))
    {
        return true;
    }
}



vector<Node> ASTAR::descending_sort(vector<Node> nodelist) {
  Node temp;
  for ( int i = 0; i < (int)nodelist.size(); ++ i ) {
    for ( int j = 0; j < (int)nodelist.size()-1; ++ j ) {
      if ( nodelist[j].cost < nodelist[j+1].cost ) {
        temp = nodelist[j];
        nodelist[j] = nodelist[j+1];
        nodelist[j+1] = temp;
      }
    }
  }
  return nodelist;
}



// initialise grid map
void ASTAR::setup_gridmap(){
  gridmap_.clear();
  grid_height_ = int(ceil(map_height_/grid_resolution_));
  grid_width_  = int(ceil(map_width_/grid_resolution_));

  cout << "grid size: width = " << grid_height_ << ", height = " << grid_width_ << endl;

  for ( int y = 0; y < grid_height_; ++ y ) {
    vector<Grid> gridx;
    for ( int x = 0; x < grid_width_; ++ x ) {
      Grid d;
      d.closed = 0; d.occupied = 0;
      d.heuristic = 0; d.expand = numeric_limits<int>::max();
      gridx.push_back( d );
    }
    gridmap_.push_back( gridx );
  }

  // set occupied flag
  int step = (int)(grid_resolution_/map_resolution_);

  for ( int y = 0; y < grid_height_; ++ y ) {
    for ( int x = 0; x < grid_width_; ++ x ) {
      cv::Point tl, dr;
      tl.x = step*x; tl.y = step*y;
      dr.x = min( map_img_.cols, step*(x+1) );
      dr.y = min( map_img_.rows, step*(y+1) );

      bool found = false;
      for ( int i = tl.y; i < dr.y; ++ i ) {
        for ( int j = tl.x; j < dr.x; ++ j ) {
          if ( (int)map_img_.at<unsigned char>(i,j) != 255 ) {
            found = true;
          }
        }
      }
      gridmap_[y][x].occupied = found == true? 1: 0;
    }
  }
}


// init heuristic
void ASTAR::init_heuristic(Node goal_node) {
  // QUESTION
  // initialise heuristic value for each grid in the map 

    // just takes the manhattan distance from the current point to the goal node (xgoal - xpoint) + (ygoal - ypoint)
    // the absolute is take so there are no negative costs

    for (int y = 0; y < gridmap_.size(); y ++)
    {
        for(int x = 0; x < gridmap_[y].size(); x++)
        {
            gridmap_[y][x].heuristic = std::abs(goal_node.x - x) + std::abs(goal_node.y - y);
            gridmap_[y][x].expand = 0;//these two i just initalise in case they havnt been already;
            gridmap_[y][x].closed = 0;
        }
    }

}

// generate policy
void ASTAR::policy(Node start_node, Node goal_node) {
  Node current;
  current = goal_node;
  bool found = false;
  optimum_policy_.clear();
  optimum_policy_.push_back( current );

  std::vector<Node> tempReverse;


  // QUESTION
  // search for the minimum cost in the neighbourhood.
  // from goal to start


  /*
   *        The method used here is very simple, it first takes a look around the current point starting from the goal node
   *        it then only focuses on those points in the grid that have been closed, after that it selects the one with the
   *        smallest expansion value. This is so it follows the path that took the least amount of expansions to get to there
   *        from the start node. After that it pushes that node onto the optimum_policy_ vector and sets the base point to
   *        that point and starts again
   */

  while (!found)
  {
      int x1 = optimum_policy_.back().x;
      int y1 = optimum_policy_.back().y;

      int minCost;
      bool firstRun = true;

      Node foundNew;
      for (int x = -1; x<=1; x++)
      {
          for(int y = -1; y<= 1; y++)
          {
              if((x == -1 && y == 1)||(x == -1 && y == -1)||(x==0 && y == 0)||(x == 1 && y == 1)||(x == 1 && y == -1))
              {
                  continue;
              }

              if(inRange((y1+y),(x1+x)))
              {

                  if (gridmap_[y1 + y][x1 + x].closed == 1)
                  {
                      if (firstRun)
                      {
                          foundNew.x = x1 + x;
                          foundNew.y = y1 + y;
                          foundNew.cost = gridmap_[y1 + y][x1 + x].expand;
                          minCost = gridmap_[y1 + y][x1 + x].expand;
                          firstRun = false;
                      }
                      else
                      {
                          if (gridmap_[y1 + y][x1 + x].expand < minCost)
                          {
                              foundNew.x = x1 + x;
                              foundNew.y = y1 + y;
                              foundNew.cost = gridmap_[y1 + y][x1 + x].expand;
                              minCost = gridmap_[y1 + y][x1 + x].expand;
                          }
                      }
                  }

                  gridmap_[y1 + y][x1 + x].closed = 0;
              }
          }
      }


      optimum_policy_.push_back(foundNew);

      if(foundNew.x == start_node.x && foundNew.y == start_node.y)
      {
          found = true;
      }
  }

  //the following block simply reverses the order of optimum_policy_

  tempReverse = optimum_policy_;

  optimum_policy_.clear();

  for (int i = tempReverse.size() - 1; i >=0; i--)
  {
      optimum_policy_.push_back(tempReverse.at(i));
  }

  tempReverse.clear();


}

// update waypoints
void ASTAR::update_waypoints(double *robot_pose) {
  waypoints_.clear();
  Waypoint w;
  w.x = robot_pose[0];
  w.y = robot_pose[1];
  waypoints_.push_back( w );
  for ( int i = 1; i < (int)optimum_policy_.size(); ++ i ) {
    w.x = grid2meterX(optimum_policy_[i].x);
    w.y = grid2meterY(optimum_policy_[i].y);
    waypoints_.push_back( w );
  }

  smooth_path( 0.5, 0.3);
//  smooth_path(0,0);




  poses_.clear();
  string map_id = "/map";
  ros::Time plan_time = ros::Time::now();
  for ( int i = 0; i < (int)waypoints_.size(); ++ i ) {
    Waypoint & w = waypoints_[i];
    geometry_msgs::PoseStamped p;
    p.header.stamp = plan_time;
    p.header.frame_id = map_id;
    p.pose.position.x = w.x;
    p.pose.position.y = w.y;
    p.pose.position.z = 0.0;
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 0.0;
    p.pose.orientation.w = 0.0;
    poses_.push_back( p );
  }
  waypoints_done_ = true;
}

// smooth generated path
void ASTAR::smooth_path(double weight_data, double weight_smooth) {
  double tolerance = 0.00001;
  // QUESTION
  // smooth paths

  waypoints_.clear();

  std::vector<Waypoint> originalPath;

  std::vector<Waypoint> previousWay;

  //filling in all the vectors with the optimum policy data in order to update it later

  for (int i = 0; i<optimum_policy_.size(); i++)
  {
      Waypoint init;
      Node temp = optimum_policy_.at(i);

      init.x = grid2meterX(temp.x);
      init.y = grid2meterY(temp.y);

      waypoints_.push_back(init);
      originalPath.push_back(init);
      previousWay.push_back(init);
  }


  double sum = tolerance + 1; //this is to get the following while loop to run the first time without adding another flag variable

  while (sum > tolerance)
  {
      sum = 0;

      //the following loop performs 3 tasks:
      //1. updates previousWay with the current waypoints
      //2. updates the current way points and places them inside waypoints
      //3. calculates the difference between the two points and adds it to the sum variable
      for (int i = 1; i<waypoints_.size() - 1; i++)
      {
          //updating previousWay with Si
          previousWay[i].x = waypoints_[i].x;
          previousWay[i].y = waypoints_[i].y;

          //updating waypoints_ with Si(new)

          waypoints_[i].x = waypoints_[i].x - (weight_data + 2*weight_smooth)*waypoints_[i].x + weight_data*originalPath[i].x +
                            weight_smooth*waypoints_[i - 1].x + weight_smooth*waypoints_[i + 1].x;


          waypoints_[i].y = waypoints_[i].y - (weight_data + 2*weight_smooth)*waypoints_[i].y + weight_data*originalPath[i].y +
                            weight_smooth*waypoints_[i - 1].y + weight_smooth*waypoints_[i + 1].y;

          //updating the difference
          sum += pow((waypoints_[i].x - previousWay[i].x),2) + pow((waypoints_[i].y - previousWay[i].y),2);
      }
  }


}

// search for astar path planning
bool ASTAR::path_search() {
  setup_gridmap();
  print_grid_props( "o" );

  Node start_node, goal_node;
  start_node.x = meterX2grid(start_[0]);
  start_node.y = meterY2grid(start_[1]);
  start_node.cost = 0;
  goal_node.x = meterX2grid( goal_[0] );
  goal_node.y = meterY2grid( goal_[1] );



  // check start of goal is occupied
  if ( gridmap_[start_node.y][start_node.x].occupied != 0 ||
       gridmap_[goal_node.y][goal_node.x].occupied != 0) {
    cout << "The goal/start is occupied\n";
    cout << "Please change another position\n";
    return false;
  }

  init_heuristic(goal_node);

  gridmap_[start_node.y][start_node.x].closed = 1;
  gridmap_[start_node.y][start_node.x].expand = 0;

  // create open list
  std::vector<Node> openList;

  // create closed List (for my reference while tracking the position)
  std::vector<Node> closedList;
  closedList.push_back(start_node);

  bool goalFound = false;

  /*
   *        The way this path search works is fairly simple, it takes the current node (starting at the start node) and looks around itself
   *        (these points are pushed onto the openList which i clear before hand in order to constrain processing to those points around it.
   *        whenever it looks around itself it ignores closed points and points with obstacles. It then picks the point that has the lowest total
   *        cost (G + H). It closes that point, updates its expansion value and pushes it to the closed list. It does this until it reaches the
   *        goal node.
   *
   *        in the event that the expansion pushes itself into a corner, it will proceed to search through all the closed points in the grid and
   *        select the one with the smallest H cost, it will then continue its search from this point
   */

  while(!goalFound)
  {
      openList.clear();

      Node currentNode;
      currentNode = closedList.back();

      //adding the nearby points into the open list
      for (int x = -1; x<=1; x++)
      {
          for(int y = -1; y<= 1; y++)
          {
              if((x == -1 && y == 1)||(x == -1 && y == -1)||(x==0 && y == 0)||(x == 1 && y == 1)||(x == 1 && y == -1))
              {
                  continue;
              }

              if(inRange((currentNode.y + y),(currentNode.x + x)))
              {
                  Node nearby;

                  nearby.x = currentNode.x + x;
                  nearby.y = currentNode.y + y;
                  nearby.cost = std::abs(start_node.x - nearby.x) + std::abs(start_node.y - nearby.y) + gridmap_[nearby.y][nearby.x].heuristic;

                  if(gridmap_[nearby.y][nearby.x].occupied == 0 && gridmap_[nearby.y][nearby.x].closed == 0)
                  {
                      openList.push_back(nearby);
                  }
              }
          }
      }

      //safety in case the search finds itself in a corner with no where else to go

      if (!openList.empty())
      {
          //sorts the list so the smallest point is at the end
          openList = descending_sort(openList);

          closedList.push_back(openList.back());

          //closing the grid point and setting the expand value
          gridmap_[openList.back().y][openList.back().x].closed = 1;
          gridmap_[openList.back().y][openList.back().x].expand = closedList.size();

          //determining if the search is done
          if(openList.back().x == goal_node.x && openList.back().y == goal_node.y)
          {
              goalFound = true;
          }
      }
      else
      {
          //in the event that the search has nowhere else to go it will execute the following code
          //this will cause it to restart the search at an already closed point that has the smallest
          //H cost and has open points around it (this will cause it to start search at the point that was
          //the closest to goal before this happened)

          int minH;
          int minx;
          int miny;
          bool firstRun = true;
          for (int ysearch = 0; ysearch < gridmap_.size(); ysearch++)
          {
              for (int xsearch = 0; xsearch < gridmap_[ysearch].size(); xsearch++)
              {
                  if (gridmap_[ysearch][xsearch].closed == 1)
                  {
                      for (int x = -1; x<=1; x++)
                      {
                          for(int y = -1; y<= 1; y++)
                          {
                              if((x == -1 && y == 1)||(x == -1 && y == -1)||(x==0 && y == 0)||(x == 1 && y == 1)||(x == 1 && y == -1))
                              {
                                  continue;
                              }

                              if(gridmap_[ysearch + y][xsearch + x].occupied == 0 && gridmap_[ysearch + y][xsearch + x].closed == 0)
                              {

                                  if(inRange((currentNode.y + y),(currentNode.x + x)))
                                  {
                                      if (firstRun)
                                      {
                                          minH = gridmap_[ysearch][xsearch].heuristic;
                                          minx = xsearch;
                                          miny = ysearch;
                                          firstRun = false;
                                      }
                                      else
                                      {
                                          if (gridmap_[ysearch][xsearch].heuristic < minH)
                                          {
                                              minH = gridmap_[ysearch][xsearch].heuristic;
                                              minx = xsearch;
                                              miny = ysearch;
                                          }
                                      }
                                  }
                              }
                          }
                      }
                  }
              }
          }

          Node startAgain;

          startAgain.x = minx;
          startAgain.y = miny;

          closedList.resize(gridmap_[miny][minx].expand);
          closedList.push_back(startAgain);
      }
  }

  cout<<"updating policy" <<endl;
  policy( start_node, goal_node );

  cout<<"updating waypoints" <<endl;
  update_waypoints( start_ );


  initialised_ = true;

  publish_plan(poses_);

  return true;
}

void ASTAR::publish_plan(const vector<geometry_msgs::PoseStamped> &path) {
  if ( !initialised_ ) {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return;
  }

  // create a message for the plan
  nav_msgs::Path path_msgs;
  path_msgs.poses.resize( path.size() );
  if (!path.empty()) {
    path_msgs.header.frame_id = path[0].header.frame_id;
    path_msgs.header.stamp = path[0].header.stamp;
  }

  for ( int i = 0; i < (int)path.size(); ++ i ) {
    path_msgs.poses[i] = path[i];
  }
  while (initialised_) {
    plan_pub_.publish( path_msgs );
  }
}





int main( int argc, char ** argv ) {
  ros::init( argc, argv, "astar_planner" );
  ros::NodeHandle nh("~");
  ASTAR astar( nh );
  ros::spin();
  return 1;
}



