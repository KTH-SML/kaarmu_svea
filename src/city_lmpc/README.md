# NAME

## Installation

...

## Usage

### ABConnect connections 

This is what we want on the same ROS Master

- (top) `/rsu/obs/target`: rsu -> obs
- (top) `/rsu/ego/target`: rsu -> ego
- (top) `/rsu/onc/target`: rsu -> onc
- (top) `/obs/state`: obs -> rsu
- (top) `/ego/state`: ego -> rsu
- (top) `/onc/state`: onc -> rsu
- (srv) `/obs/enable_vehicle`: obs .. rsu
- (srv) `/ego/enable_vehicle`: ego .. rsu
- (srv) `/onc/enable_vehicle`: onc .. rsu

A topic is one way and require only one connection.

A service is two topics for each way and requires therefore two connections.
The list below has topic ids. The service id is the first in a pair of topic ids,
first for the request and second for the response, i.e. topic request id is the
same as service id and implcitly the response must then be the following integer,
e.g. 6 (request) and 7 (response).

00. `/obs/state`: obs -> rsu
01. `/ego/state`: ego -> rsu
02. `/onc/state`: onc -> rsu
03. `/rsu/obs/target`: rsu -> obs
04. `/rsu/ego/target`: rsu -> ego
05. `/rsu/onc/target`: rsu -> onc
06. `/obs/enable_vehicle`: obs <- rsu
07. `/obs/enable_vehicle`: obs -> rsu
08. `/ego/enable_vehicle`: ego <- rsu
09. `/ego/enable_vehicle`: ego -> rsu
10. `/onc/enable_vehicle`: onc <- rsu
11. `/onc/enable_vehicle`: onc -> rsu

How relevant topics/services will look like for RSU:

00. sub `/abconnect/obs/state`
03. pub `/abconnect/obs/target`
06. pub `/abconnect/obs/srv/req/enable_vehicle`
07. sub `/abconnect/obs/srv/res/enable_vehicle`
01. sub `/abconnect/ego/state`
04. pub `/abconnect/ego/target`
08. pub `/abconnect/ego/srv/req/enable_vehicle`
09. sub `/abconnect/ego/srv/res/enable_vehicle`
02. sub `/abconnect/onc/state`
05. pub `/abconnect/onc/target`
10. pub `/abconnect/onc/srv/req/enable_vehicle`
11. sub `/abconnect/onc/srv/res/enable_vehicle`

How relevant topics/services will look like for SVEA:

- [00,01,02] pub `/abconnect/rsu/state`
- [03,04,05] sub `/abconnect/rsu/target`
- [06,08,10] sub `/abconnect/rsu/srv/req/enable_vehicle`
- [07,09,11] pub `/abconnect/rsu/srv/res/enable_vehicle`

### Docker

1. Build with `util/build`
2. Enter the docker container with `util/run`
3. Run example application inside the container with `roslaunch city_lmpc sml.launch`
