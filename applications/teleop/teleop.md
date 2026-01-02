# teleop

## Find dev/input/eventX
How to setup teleop node with right keyboard:

Run this in one terminal:
```
for f in /dev/input/event*; do
  echo "Testing $f"
  sudo timeout 2 cat $f | hexdump | head
done
```
press keys on your keyboard: he file that produces binary spam only when you press keys is your keyboard.

Another way is to check:
```
cat /proc/bus/input/devices
```
and keep note of the `Handlers=sysrq kbd leds eventX`

Verify that `eventX` is the right keyboard by running
```
sudo cat /dev/input/eventx
```
and then pressing keys: if you see binary garbage is the correct device.

## update teleop_node.cpp

Temporary:
```
sudo chmod a+rw /dev/input/eventX
```

and in the constructor change:
```
input_fd_ = open("/dev/input/eventX", O_RDONLY | O_NONBLOCK);
```
