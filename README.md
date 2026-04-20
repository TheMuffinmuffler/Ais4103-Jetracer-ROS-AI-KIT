
##################### Planner and Map location #####################

- The map used to test the path planner (A*) is "Ais4103-Jetracer-ROS-AI-KIT/my_map.pgm" and 

- The file constaining the planner and visualizing it in Rviz is "Ais4103-Jetracer-ROS-AI-KIT/src/ris/src/grid_planner/plan_testRviz.cpp"
  
I modified the cost function by assigning a higher cost to cells near obstacles. I also applied inflation, (expanded obstacles by a certain radius so that nearby cells become more costly). As a result, the generated path is more centered and safer, although it is still not perfectly optimal.



##################### Screenshots from Planner #####################

Note: The grid shown in the images does not use the original resolution. The actual resolution is 0.05 m per cell, but a larger resolution is used in RViz for better visualization.


----- Path obtained with 4-connectivity planner -----

<img width="471" height="551" alt="image" src="https://github.com/user-attachments/assets/16f4ab9e-c79d-43f8-adbc-c92c4c6d1961" />


----- Path obtained with 8-connectivity planner -----

<img width="353" height="545" alt="image" src="https://github.com/user-attachments/assets/326722ad-8033-40d8-9f21-d0848a731432" />


----- Path obtained with 8-connectivity planner, inflation radius = 7  -----

<img width="394" height="544" alt="image" src="https://github.com/user-attachments/assets/da64b186-fffc-4f6b-b2a5-31ed0ac20a68" />


----- Path obtained with 8-connectivity planner, inflation radius = 10  -----

<img width="398" height="543" alt="image" src="https://github.com/user-attachments/assets/8bddbe5a-ec06-42c7-8105-491182c37bb2" />
