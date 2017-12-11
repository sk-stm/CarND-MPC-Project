# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Build
I modified the _CMakeLists.txt_ such that I don't have to install the uws-library with sudo, I rather just link specifically to its location on my system. You should just restore the original file to make it compile on your system again.

## Run
_cwd_ needs to be the project directory since it loads the `params.json` file and expects it within _cwd_, so run `./build/mpc`