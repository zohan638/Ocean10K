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


## Interfacing with Haptic Controllers

1. Clone the chai3d wrapper:
```
git clone
https://github.com/manips-sai-org/chaiHapticdeviceRedisDriver.git
```

2. Install all dependencies:
```
sudo apt-get install cmake redis redis-server libhiredis-dev libjsoncpp-dev
```

3. From the source directory, create a build directory and make:
```
cd chaiHapticdeviceRedisDriver
mkdir build && cd build
cmake .. && make -j4
```

4. Start the Redis server:
```
redis-server
```

5. Plug in the haptic device to your device (e.g. laptop) and power it on.

6. From `chaiHapticdeviceRedisDriver/build`, run:
```
sudo ./chai_devices_redis_server
```


## Running the Project
1. Start Redis server (if not already running):
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
