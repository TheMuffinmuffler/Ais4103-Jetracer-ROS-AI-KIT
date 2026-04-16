
##################### Planner and Map location #####################

- The map used to test the path planner (A*) is "Ais4103-Jetracer-ROS-AI-KIT/my_map.pgm" and 

- The file constaining the planner and visualizing it in Rviz is "Ais4103-Jetracer-ROS-AI-KIT/src/ris/src/grid_planner/plan_testRviz.cpp"
  
I modified the cost function by assigning a higher cost to cells near obstacles. I also applied inflation, (expanded obstacles by a certain radius so that nearby cells become more costly). As a result, the generated path is more centered and safer, although it is still not perfectly optimal.
