1. Download and install g2o or gtsam, follow their instuctions 

    g2o: https://github.com/RainerKuemmerle/g2o   
    gtsam_official: https://borg.cc.gatech.edu/download.html  
    gtsam3.1: https://github.com/borglab/gtsam  
    gtsam4.0:  https://bitbucket.org/gtborg/gtsam.git

2. Change the directories of g2o/gtsam in the CMakeLists.txt  
    change the lines:   
    set(G2O_ROOT /home/davidz/work/3rdlib/g2o/install/usr/local)  
    set(GTSAM_ROOT /home/davidz/work/3rdlib/gtsam/build)  
    to your folder where g2o/gtsam is installed. 

3. You do not need to compile both of them, choose one and comment the other.     
   Or you can compile and run both of them if you like. 

4. make the project following:  
    mkdir build     
    cd build  
    cmake ..  
    make  

5. After compilation, you can run it:  
    cd bin  
    ./test_g2o ../data/vro_results.log  or  
    ./test_gtsam ../data/vro_results.log  
  
  You should see the results similar to that demoed in class.   
  
  GOOD LUCK! 
  



