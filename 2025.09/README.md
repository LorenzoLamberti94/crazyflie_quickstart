# Crazyflie Quickstart
_Start developing your algorithms !_

Basic functionalities implemented:
- take-off
- landing
- setpoints (relative vs. Abs mode)
- remote control with GUI parameters for starting the mission

### Flashing:
change inside the makefile the CRAZYFLIE_BASE path to your local crazyflie-firmware/ location.

To flash your code on the cf (broadcast mode):
```
conda activate cfclient_2025.09
rm -rf build/
make -j all
make cload
```

Point-to-point flashing:
```
conda activate cfclient_2025.09
rm -rf build/
make -j all
CLOAD_CMDS="-w radio://0/80/2M" make cload
```


### Start Mission
open the CF client
```
conda activate cfclient_2025.09
cfclient
```
- go to Parameters tab
- set START_STOP/fly parameter to 1 (takes off)
- set the forward speed with DRONET_PARAMS/velocity

## Git tags

_Tested with following tags :_
- crazyflie-firmware:
- crazyflie-clients-python:
- crazyflie-lib-python:
