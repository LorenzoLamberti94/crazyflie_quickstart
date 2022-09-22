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
make clean all
make cload 
```


Point-to-point flashing: 
```
make clean all
CLOAD_ARGS="-w radio://0/80/2M/E7E7E7E7E7" make cload
```

Other flashing utils:
```
be fast !
make -j 8 clean all 
make -j 8 clean all && make cload
```


### Start Mission
open the CF client
```
cfclient
```
- go to Parameters tab
- set START_STOP/fly parameter to 1 (takes off)
- set the forward speed with DRONET_PARAMS/velocity

## Git tags

_Tested with following tags :_
- crazyflie-firmware: 2022.01
- crazyflie-clients-python: 4eb749a94571ebd0b744d23207cb916560460ad2
- crazyflie-lib-python: 7b1fc38358690fe887866347bf0e502f14d4e4fc
