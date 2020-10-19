## RRT Star

### Setup
1. Create a Python 3 Virtual Environment in the root directory of the project
```sh
python3 -m virtualenv venv
```
2. Activate the Virtual Environment (Fish Shell Example Below)
```fish
. venv/bin/activate.fish
```
3. Upgrade `pip` and `setuptools`
```sh
python3 -m pip install -U setuptools pip
```
4. Install the projects requirements
```sh
python3 -m pip install -r requirements.txt
```
5. (Optional) Install the pre-commit hook (requires clang-format and cppcheck to be installed on the system)
```sh
python3 -m pre_commit install
```

### Building
1. Setup the build system (GCC Example) in the project's root directory
```sh
meson build-gcc
```
2. Change to the build directory
```sh
cd build-gcc
```
3. Compile the project
```sh
meson compile
```

## Usage
```
cd <project_root_dir>/build*
./example
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
- This implementation currently assumes that if the costmap has a coordinate it is not accessible.
- It also assumes that this should be used in a single threaded environment. Futures, locks, and threading could be added to increase performance.
