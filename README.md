# Ocean 10K

[![HitCount](https://hits.dwyl.com/zohan638/CS225A-Project.svg?style=flat-square)](http://hits.dwyl.com/zohan638/CS225A-Project)


## Dependencies
This project depends on sai2 libraries. The install instructions can be found
[here](https://github.com/manips-sai-org/OpenSai).

## Build Instructions
Navigate to the `build` directory, then make by:
```
cd build
cmake .. && make -j4
```

## Running the Project
1. Start Redis server:
```
redis-server
```

2. From the `bin` folder, navigate to the directory of the application to run:
```
cd bin/ocean1
```

3. Run the simulation:
```
./simviz_ocean1
```
4. From another terminal, run:
```
./controller_ocean1
```
