Simulation Lab Core
===================

Simulation Lab (SL) is a robotics simulator and a real time control engine
that is used by the IAS lab. It was originally developed by Stefan Schaal 
and colleagues at USC, and it is used at various institutions
(e.g. Max Planck Institute in Tuebingen, CMU in the USA, ATR in Japan, etc.),
allowing for easy cooperation and joint projects between those labs.

SL can be used in two different modes: *simulation* and *real time control*.
In simulation, new tasks and methods can be tried out on models of the 
available robots before testing on the actual real system, thereby making 
the setup of potentially dangerous (to the robot or its environment) or 
time consuming experiments more comfortable. However, SL and its concept 
may take some time getting used to in the beginning.
SL runs on Linux and MacOS, and it is currently distributed internally.


Quick Links
-----------

* [Installation Guide](#installation-guide)
* [SL book](http://www-clmc.usc.edu/publications/S/schaal-TRSL.pdf)
* [Documentation](docs/Readme.md)


Installation Guide
------------------

### Mac OS

* We recommend to use [Homebrew](http://brew.sh/) to install packages
  `/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"`

> All packages installed using `brew install` go into `/usr/local`. 
You should never need to use `sudo` with `brew`.

* Install CMake  
  `brew update`
  `brew install cmake`
* Install [XQuartz](http://www.xquartz.org/) (required to show graphics)
* Install freeglut
  `brew install homebrew/X11/freeglut`
* In order for SL to find X11 and freeglut, you need to create a symlink 
to the folder with these libraries  
  `mkdir /opt/local`  
  `ln -s /opt/X11/lib /opt/local/lib`

> Make sure that `/opt/X11/bin` is in your `PATH` by running `echo $PATH`.
If it is not, add line `export PATH=$PATH:/opt/X11/bin` to `~/.profile`
and restart Terminal.

* Install readline  
  `brew install readline`
* Change the shared memory size by editing `/etc/sysctl.conf` and adding  
  `sudo nano /etc/sysctl.conf`  
  `kern.sysv.shmmax=2147483648`  
  `kern.sysv.shmall=524288`  
  `kern.sysv.shmmni=128`  
  `kern.sysv.shmseg=32`  

* For older version of OSX (prior to 10.4)  
  `sudo nano /etc/sysctl.conf`  
  `kernel.shmmax=2147483648`  
  `kernel.shmall=524288`  
  `kern.sysv.shmmni=128`  
  `kern.sysv.shmseg=32`  

> If you want to know more about shared memory on Mac OS X, read 
> * [Configuring Shared Memory on Mac OS X](http://www.spy-hill.com/~myers/help/apple/SharedMemory.html) 
> * [Adjusting Shared memory segment values](https://support.apple.com/en-us/HT4022).

* Restart the computer. Check if you configured shared memory correctly
by running `sysctl kern.sysv.shmmax`. Similarly, you can check other variables.


### Ubuntu

* Install the following dependencies  
  `sudo apt-get install libncurses5-dev libreadline6-dev freeglut3-dev libxmu-dev cmake cmake-curses-gui libedit-dev libstd++-6-dev clang xterm`
* Change the shared memory size  
  `sudo nano /etc/sysctl.conf`  
  `kernel.shmmax=2147483648`  
  `kernel.shmall=2147483648`  
* Restart the computer


### Compiling and Running SL

* Go to your sl folder and run  
  `mkdir build`  
  `cd build`  
  `ccmake ..`  
* The CMake interface up will open and you can edit compiling preferences.
  You can choose which Matlab version to use or which task to compile for 
  each robot. The first time do  
  * `c` (configure)  
  * select the robots to build (eg, `BUILD_barrett`)  
  * `c`  
  * select the tasks to build (eg, `barrett_gen_matlab`)  
  * `g` (generate)  

> It can happen that the Matlab root is not automatically recognized and
> you receive an error at the second `c`. If that happens, press `e` to exit 
> the message and manually set the Matlab root. Also, if you want to compile 
> a robot task which needs Matlab, you have to enable the Matlab interface 
> (eg, `BUILD_MATLAB_INTERFACE_barrett`) or the task will not be compiled 
> (you will not receive any compilation from CMake, though).

* It is recommended to change `CMAKE_C_COMPILER` and `CMAKE_CXX_COMPILER` 
  to `clang` and `clang++` by toggling the advanced preferences (`t`)
* You are then ready to build  
  `make install`
* You can also build a single robot from its folder  
  `cd sl/build/barrett`  
  `make install`  
* Finally, to run a robot  
  `cd sl/build/barrett`  
  `./xbarrett`


### Optional Compillation and Running Steps

#### MATLAB 2013a on OSX

Do `ccmake` press `t` and add to `C_FLAGS`  
  `-Dchar16_t=uint16_T`

#### Remove Real Time Limits

* `sudo nano /etc/security/limits.conf`
* Add anywhere the following two lines  
  `* - rtprio 99`  
  `* - memlock unlimited`
