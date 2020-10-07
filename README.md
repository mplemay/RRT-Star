### RRT Star Implementation
## Building
```
cd <project_root_dir>
mkdir -p build
cd build/
cmake ..
make
```
## Usage
```
cd <project_root_dir>/build
./zebra_assignment_part_2
Enter the size of the boarder square: 100
Enter the step size (i.e. double): 5.0
Enter the size of the neighbor search radius (i.e. double): 10.0
Enter the amount of time the search should continue for after getting an initial path (in milliseconds): 100000
Where should the algorithm search start point be?
Enter the x coordinate: -1
Enter the y coordinate: -1
Where should the algorithm search goal point be?
Enter the x coordinate: 100
Enter the y coordinate: 100
Initial path (size = 37): (100,100) (99,100) (96,99) (96,98) (91,94) (90,94) (83,87) (82,87) (75,80) (74,80) (71,77) (70,77) (65,75) (60,69) (59,68) (53,61) (52,61) (51,58) (51,57) (45,53) (45,52) (40,46) (39,45) (37,42) (37,41) (32,34) (32,33) (30,28) (23,22) (23,21) (17,16) (16,15) (10,8) (9,8) (1,4) (1,3) (-1,-1) 
Refined path (size = 29): (100,100) (95,93) (90,88) (84,84) (83,84) (79,79) (78,79) (73,74) (67,67) (61,60) (60,60) (56,54) (51,49) (50,49) (43,42) (42,42) (35,35) (29,30) (26,26) (25,26) (18,21) (17,21) (10,15) (9,14) (6,9) (5,8) (1,4) (1,3) (-1,-1) 
```

## Notes
- This program was optimized for memory usage. Thus, it uses a sparse costmap, limited chaching of results, etc.
- The cost map is templated so that different values types can be used.
- Currently the RTTStar and Costmap2D are header-only they could be made libraries if there is
- This implementation currently assumes that if the costmap has a coordinate it is not accessible.
- It also assumes that this should be used in a single threaded environment. Futures and locks could be added with an added increase in code complexity.

