INSTALLATION:
1. Install Chai3d: git@github.com:manips-sai-org/chai3d.git
2. Install sai2-urdfreader: git@github.com:manips-sai-org/sai2-urdfreader.git
3. Install sai2-model: git@github.com:manips-sai-org/sai2-model.git
4. Install sai2-graphics: git@github.com:manips-sai-org/sai2-graphics.git
5. Install sai2-simulation: git@github.com:manips-sai-org/sai2-simulation.git (branch shameek_sp)
6. Build 
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make
$ cd ..
7. Import graphics files from robo:/afs/cs.stanford.edu/group/manips/sim_resources/gold_mining/ to resources/
8. Run
$ cd bin
$ ./01-cylinders
