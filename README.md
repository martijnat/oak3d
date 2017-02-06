# oak3d

Oak3d is a command line application that draws 3d models using
terminal escape codes.

![Screenshot](screenshot.png)

# Usage

```
./oak3d.py [FILE]
```

Where FILE points to a .obj file. There are example models in the "models" folder.

# Notes

This application requires

- A terminal with full truecolor support (many terminals only support up to 256 colors)
- Model files with normal vectors

If you want to render models on a terminal that do not supports this,
check out [rice3d](https://github.com/martijnat/rice3d), it has a
simpler shading model but work with a larger variety of models and
terminals.
