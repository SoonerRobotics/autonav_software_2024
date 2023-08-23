# autonav_software_2024

Software for the 2024 [Intelligent Ground Vehicle Competition](http://www.igvc.org/), AutoNav challenge, the **Robot**.  
We are using [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on [Ubuntu 22.04](https://releases.ubuntu.com/22.04/).

## VSCode

To edit the software with Visual Studio Code, please install the ros extension and open VSCode through the command line via `code` after running all steps under [Building](#building). To get proper intellisense for C++, create the following file: `.vscode/c_cpp_properties.json`
```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```

## Style
All folder names including packages should be in `snake_case`
Avoid most abbreviations
Tab length is 4 spaces (default)

CMakeLists.txt and package.xml should follow the patterns found in [format_packages](https://github.com/SoonerRobotics/autonav_software_2024/tree/feat/particle_filter/scripts/format_package)

### Python
Python files should be written in the [PEP-8 style](https://peps.python.org/pep-0008/)

### C++
file names should be in `snake_case`
class names should be in `UpperCamelCase` (to agree with [rclcpp](https://docs.ros2.org/foxy/api/rclcpp/index.html))
function names should be in `camelCase`

