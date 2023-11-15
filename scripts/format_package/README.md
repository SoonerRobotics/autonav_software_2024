Run this script using python3 inside of a ros package that uses python and C++ nodes to generate CMakeLists.txt and package.xml that are formatted for this purpose.

navigate to `~/your/ros/package/`

enter `python3 ~/path/to/format_package/format_package.py`

You'll still want to update the package name in each file. When you run `colcon build` you'll see that the packages have the default name `add_your_package_name_in_cmakelists_and_packagexml` until you edit them, one of them will build though.

The script sets the package name to `add_your_package_name_in_cmakelists_and_packagexml`, the description to `TODO: Package description`, the maintainer email to `addyour@email.here`, and the license to `MIT`.


