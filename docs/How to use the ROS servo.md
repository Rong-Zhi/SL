How to use the ROS servo
------------------------


### 1. Enable building the ROS servo

In order to enable building of the ROS servo for your robot, add the following
piece of code into your `CMakeLists.txt` file of your robot.

```CMake
set(SRCS_XROS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SL_user_ros.cpp
)
```


### 2. Write your code

Copy `SL/srcUserTemplates/SL_user_ros.c` to the folder `YOUR_ROBOT/src`.
Change its extension to `cpp`. Here is an example file. You can put whatever
you want into `init_user_ros` and `run_user_ros` functions.

```C++
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "SL_system_headers.h"
#include "SL.h"
#include "SL_ros_servo.h"

namespace sl2ros {
  class Listener {
  public:
    Listener(ros::NodeHandle& nh) {
        sub = nh.subscribe("/ROS_TOPIC", 1, chatterCallback)
    };
    virtual ~Listener(ros::NodeHandle& nh) {};

    static void chatterCallback(const std_msgs::String::ConstPtr msg) {
      printf("%s", msg->data.c_str());
    }

    void init() {
            printf("Listening to topic");
    }

  private:
    ros::Subscriber sub;
  };
}

static ros::NodeHandle nh("");
static sl2ros::Listener ros_listener(nh);

int init_user_ros(void) {
  ros_listener.init();
  return TRUE;
}

int run_user_ros(void) {
  return TRUE;
}
```


### 3. Build and run

* From the terminal, source your ROS environment (e.g. `source /opt/ias_ros/setup.sh`).
* In the CMake configuration, make sure that the option `BUILD_SL_ROS` is `ON`.
* `make install` as usual.
* Run with `-ros` option (e.g. `./xdarias -ros`)


### 4. Run default publisher

There is a built-in publisher in SL, that streams joint data and end-effector
state. You can enable it by adding the line

    default_publisher_enabled   1

to `YOUR_ROBOT/config/ParameterPool.cf`.
