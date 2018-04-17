This code follows TDD approach and is structured as follows:

## build folder:
    --> Used for building the c++ project with all dependencies included in the CMakeLists.txt.
    --> Use "cmake .." to build dependencies with libraries and directories.
    --> Followed by "make" to build C++ code.
    --> Successful building of the code generates the following two executables: "main" and "cylinderSegTest"
    --> "main" executable is located in the "src" folder inside build and is used to run and generate the cylinder as well as plane .pcd files.
    --> "cylinderSegTest" executable on running checks if the testcases are passing/failing.
    
## src folder:
    --> Contains cylinder segmentaion folder which includes the code required for obtaining a cylinder pcd file from a given compressed binary pcd file.
    --> main.cpp calls the cylinder segmentation code and as further libraries are developed these can be called from main.cpp.
    
## tests folder:
    --> Contains basic testcases for the cylinder segmentation library.
    
## Dataset folder:
   --> Contains the input and output pcd files.
   --> Web folder consists of three.js libraries, javascript and render.html file to see the final result on Browser.
   
## To see the results on web:
   
   First a local server is started using : http-server -c-1 inside Dataset folder from terminal.
   Then navigating to www/render.html displays the results.
    
