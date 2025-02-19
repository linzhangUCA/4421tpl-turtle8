# ROS 2 Package
## Objectives
- Create a ROS 2 package.
- Link a ROS executable to a Python script.
- Create a publisher **node** for publishing a **topic** with [`String`](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html) **message**.

## Requirements: 
1. Create a ROS package with name `r2_pkg` using the CLI.
1. (30%) Create a ROS executable associate to the Python script. Refer to this [tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html). 
> Hint:
> ```console
> cd <ros workspace location>/src
> ros2 pkg create --build-type ament_python --node-name <executable name> <package name>
> ``` 

2. (40%) Complete the executable python file to publish a [String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html) **message** under the **topic** `/greeting`. The contents of the **message** can be any text followed by an index number (e.g. "Hello World 123"). The `/greeting` **topic** should be published at the frequency of 10 Hz. Reference to this [tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

3. (20%) Build this package using `colcon`. Refer to this [tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html). 
> Hint:
> ```console
> cd <ros workspace location>/
> colcon build
> ```

Verify if the executable python script is recognizable by ROS.
> Hint:
> ```console
> ros2 run <package name> <executable name>
> ```

4. (10%) Stamp the package with your own signature by editing `package.xml` and `setup.py`. Fill `<maintainer>`, `<maintainer_email>`, `<description>` or any other fields with appropriate information. Refer to this [tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html). 

## Troubleshooting:
- Make sure your environment is correctly setup by following this [tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
- Make sure you have sourced your workspace's configuration before `ros2 run ...`
```console
source <ros workspace location>/install/local_setup.bash
```
- Make sure you have edited `setup.py` and include the Python file in **'console_scripts'**.
