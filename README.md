<img src="./doc/images/IWT.png" align="right"
     title="IWT logo" width="184" height="55">

# nimbus_pnp_demo

Test to demontrate the pick and place application using [Azure Kinect Dk](https://azure.microsoft.com/en-us/services/kinect-dk/). Later the algorithm will be ported to use Nimbus camera. 

|Branch    | ROS Distro | Status    |
|----------|------------|-----------|
|master    | Melodic    |[![Build Status](https://travis-ci.org/prachandabhanu/nimbus_pnp_demo.svg?branch=master)](https://travis-ci.org/prachandabhanu/nimbus_pnp_demo)|

# Setting up the evnironment
Both sourcing and executing the script will run the commands in the script line by line, as if you typed those commands by hand line by line.

The differences are:
* When you execute the script you are opening a new shell, type the commands in the new shell, copy the output back to your current shell, then close the new shell. Any changes to environment will take effect only in the new shell and will be lost once the new shell is closed.
* When you source the script you are typing the commands in your current shell. Any changes to the environment will take effect and stay in your current shell.

Use source if you want the script to change the environment in your currently running shell. use execute otherwise.
Answer is copied from [Stack Exchange](https://superuser.com/questions/176783/what-is-the-difference-between-executing-a-bash-script-vs-sourcing-it)

`source nimbus_env.sh`

Read Documentation
1. [Cloud](https://github.com/prachandabhanu/nimbus_pnp_demo/blob/master/doc/rosdoc/detector/doc/html/index.html)