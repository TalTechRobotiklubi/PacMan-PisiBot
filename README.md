# Pac-Man project -- Pisibot Controller Atmel

These are drivers for robots that are used in the 
[Pac-Man project](https://github.com/TalTechRobotiklubi/Pacman).

## Team
* Project lead: Erki Meinberg
* Machine vision: Märtin Laanemets & Oliver Paljak
* Pisibot v5 drivers (this repository): Oliver Paljak
* Pisibot v6 electronics+drivers: Märtin Laanemets

## IMPORTANT -- About dependencies
This code depends on the Robotics Club Pisibot drivers (written by Rain Ellermaa 
and other TTÜ Robotiklubi members) that are not included in this repository. If 
you want to build the repository, include the dependencies in a folder called
"drivers".

## About compiling
To compile run the following commands:
```
mkdir build
cd build
cmake ..
make
```
### Known errors
For some reason latest versions of avr-binutils has a library called
"libctf.so.0" missing. The version that worked without problems is 2.33.1-1.
If this problem is not resolved in the future, just downgrade the avr-binutils
(the 2.33.1-1 package and its signature file has been included in this
directory: avr-binutils-2.33.1-1-x86_64.pkg.tar.xz).

Downgrading on arch linux:
```
sudo pacman -U avr-binutils-2.33.1-1-x86_64.pkg.tar.xz
```

Some vague information on this:
[direct link](https://archlinuxarm.org/forum/viewtopic.php?t=14303) and
[archived link](https://archive.is/ulgy2)
