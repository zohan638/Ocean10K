# Ocean 10K

[![HitCount](https://hits.dwyl.com/zohan638/CS225A-Project.svg?style=flat-square)](http://hits.dwyl.com/zohan638/CS225A-Project)
![Build Status](https://github.com/zohan638/CS225A-Project/actions/workflows/c-cpp.yml/badge.svg)

## Dependencies
This project depends on sai2 libraries. The install instructions can be found
[here](https://github.com/manips-sai-org/OpenSai).


## Build Instructions
Create a `build` directory, then make by running:
```
mkdir build
cd build
cmake .. && make -j4
```

## Changing Sai2Graphics Instructions
In Sai2Graphics.cpp, under 
```
void Sai2Graphics::renderGraphicsWorld()
```
You will need to comment out the lines 
```
setCameraPose(camera_name, camera_pos, camera_up_axis, camera_lookat_point);
```
and
```
render(camera_name);
```
Additionally, in Sai2Graphics.h, you will also need to move
```
void render(const std::string& camera_name);
```
from private to public.


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

6. Calibrate the device. For example, for Falcon Novint, gently pull the
controller out to its limit, press the HOME button, then release it.

7. From `chaiHapticdeviceRedisDriver/build`, run:
```
sudo ./chai_devices_redis_driver
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
5. The haptic device should return to its HOMED state. Press the HOME button
without disturbing its HOMED location. Once the button is pressed, you can
start interacting with it normally.

6. You can now interact with the environment!

![screenshot](./assets/screenshot.jpeg?raw=true)
