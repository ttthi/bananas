# Detecting planes from pointclouds

## Building

```sh
cmake -G Ninja -S . -B build
cmake --build build
```

## Usage

### Detecting planes

Download the pointclouds from [Google
Drive](https://drive.google.com/drive/folders/1UIa8giKCnrEpAmgxmNaO06Inrz5LZaXd?usp=drive_link)
and place them in a directory called `basic-clouds`. Run either
`./build/box_detection` or `./build/box_detection2`. Zoom out by scrolling down
to see the pointclouds with the detected planes.
