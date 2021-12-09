<h1 align="center"> Pozyx_UWB </h1>

## How to setup

First, install the pypozyx and requests

```
$ sudo pip install pypozyx requests         \\ Python 2.7
```
or
```
$ sudo pip3 install pypozyx requests        \\ Python 3.4
```

## How to Run

First, to get permission once (given your device is on port ACM0)
```
$ sudo chmod +x /dev/ttyACM0
```

Second, you can check your pozyx device
```
$ sudo python checking_pozyx.py
```




## Reference

- [Pypozyx](https://pypozyx.readthedocs.io/en/develop/index.html)
- [Github](https://github.com/pozyxLabs/Pozyx-Python-library)
- [Pozyx](https://docs.pozyx.io/index.html)


