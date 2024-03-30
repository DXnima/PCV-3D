# PCV-3D

#### 介绍
A Method for Combining Point Cloud and Video Data to Build 3D Realistic Models

#### Software Architecture
VS2016 | PCL 1.12.1 | CGAL 5.3.1

**Main Idea**
Use RANSAC plane fitting to find the plane equation

Calculate the farthest and nearest points

Calculate the front and back plane equations based on the farthest and nearest points and the plane equation

Use non-coplanar planes to find the intersection point

#### Installation

CMake is required for all

1.  Environment Installation (Windows)

[win10+vs2019+pcl1.11.0 Installation Tutorial](./doc/win10+vs2019+pcl1.11.0.md)

Configure environment variables

```
# New environment variables
PCL_ROOT=G:\PCL 1.12.1
BOOST_ROOT=G:\PCL 1.12.1\3rdParty\Boost
QHULL_BIN=G:\PCL 1.12.1\3rdParty\Qhull\bin
CGAL_DIR=G:\CGAL-5.3.1
GMP_LIBRARY=G:\CGAL-5.3.1\auxiliary\gmp\lib

# Environment variables in PATH
%PCL_ROOT%\bin
%PCL_ROOT%\3rdParty\VTK\bin
%PCL_ROOT%\3rdParty\Boost\lib
%PCL_ROOT%\3rdParty\FLANN\lib
%PCL_ROOT%\3rdParty\OpenNI2\Redist
%CGAL_DIR%\auxiliary\gmp\lib
%QHULL_BIN%
```

2.  Environment Installation(ubuntu)

   Install PCL、gmp、mpfr
```
 sudo apt-get install libpcl-dev libgmp-dev libmpfr-dev
```

  Install CGAL-5.3.1
```
wget https://github.com/CGAL/cgal/releases/download/v5.3.1/CGAL-5.3.1.tar.xz
tar -xvf CGAL-5.3.1.tar.xz
```

Configure environment variables

Edit profile
```
vi /etc/profile
```

Add environment variables
```
CGAL_DIR=/root/CGAL-5.3.1
GMP_LIBRARY=$CGAL_DIR/auxiliary/gmp/lib
PATH=$PATH:$GMP_LIBRARY
export PATH CGAL_DIRY
```

Update environment variables
```
source /etc/profile
```
Note: If it doesn’t work, restart and check if the environment variables are configured correctly

#### Usage Instructions

1. Start the project in release x64Configure props to start as per the [tutorial](./doc/win10+vs2019+pcl1.11.0.md)

2. CMake method (command)

   ```
   cd PCLTest
   mkdir build && cd build
   cmake -DCGAL_DIR=/root/CGAL-5.3.1 -DCMAKE_BUILD_TYPE=Debug ..
   make
   ```
   
3. CMake method (cmake-gui)

Set the source code path and the compiled path

![readme_1](doc/readme_1.png)

Click Configure, select the VS version and architecture

![readme_2](doc/readme_2.png)

Click Generate to create the VS project

![readme_3](doc/readme_3.png)

Finally, open “ALL_BUILD” in VS to generate

![readme_4](doc/readme_4.png)

!!! Set PCLTest as the startup item !!!

Error corrections:
1.Macro definitions are invalid

![readme_5](doc/readme_5.png)

Modify the PCLTest properties -> C/C++ -> Preprocessor -> Preprocessor definitions to:

![readme_6](doc/readme_6.png)

```
BOOST_USE_WINDOWS_H
NOMINMAX
_CRT_SECURE_NO_DEPRECATE
```
![readme_7](doc/readme_7.png)

2.LNK1104: unable to open file“libboost_thread-vc142-mt-gd-x64-1_78.lib”

![readme_8](doc/readme_8.png)

Modify the PCLTest properties -> VC++ directories -> Library directories to the Boost/lib under PCL:

![readme_9](doc/readme_9.png)

![readme_10](doc/readme_10.png)

3.C2001 error: newline in constant (often occurs in std::cout)

Method 1: C/C++ -> All Options -> Additional Options Input:
```
/utf-8 %(AdditionalOptions)
```
![readme_11](doc/readme_11.png)

Method 2: Save the error file as UTF-8 encoding

4.Missing qhull_rd.dll during compilation

Find the file and put it into the debug folder at the PCL installation path \3rdParty\Qhull\bin\qhull_rd.dll

#### Contributing

Fork this repository
Create a new Feat_xxx branch
Commit your code
Create a new Pull Request