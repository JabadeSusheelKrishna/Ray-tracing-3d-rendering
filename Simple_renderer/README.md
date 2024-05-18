# Introduction to Computer Graphics (CS7.302, IIIT Hyderabad)
This is the base code that is to be used for all assignments in the above course. <br>
Some sample scenes are given in the <a href="https://github.com/cs7-302-graphics/scenes">cs7-302-graphics/scenes</a> repo.

Instructions to clone, compile and run are given below.

## Cloning this repo
```git clone --recursive https://github.com/cs7-302-graphics/simple_renderer```
Do not forget the `--recursive` flag!

## Compiling
```
mkdir build
cd build
cmake ..
```

If you are on windows, this should create a Visual Studio solution ```cs7302.sln``` in the build folder. Open it and compile. \
If you are on linux/mac, you will need to additionally run the following to compile:

```
make -j8
```

## Running
The path to scene config (typically named `config.json`) and the path of the output image are passed using command line arguments as follows:
```bash
./build/render <scene_path> <out_path>
```