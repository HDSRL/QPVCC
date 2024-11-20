cd build && cmake .. && make -j8
./Run_Mujoco  ../params/LL_w_CLF.txt ../params/Walking_params.txt 
cd ..