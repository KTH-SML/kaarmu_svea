# NAME

## Installation

...

## Usage

### ABConnect connections 

This is what we want on the same ROS Master

- (top) `/corin/obs/target`: corin -> obs
- (top) `/corin/ego/target`: corin -> ego
- (top) `/corin/onc/target`: corin -> onc
- (top) `/obs/state`: obs -> corin
- (top) `/ego/state`: ego -> corin
- (top) `/onc/state`: onc -> corin
- (srv) `/obs/enable_vehicle`: obs .. corin
- (srv) `/ego/enable_vehicle`: ego .. corin
- (srv) `/onc/enable_vehicle`: onc .. corin

A topic is one way and require only one connection.

A service is two topics for each way and requires therefore two connections.
The list below has topic ids. The service id is the first in a pair of topic ids,
first for the request and second for the response, i.e. topic request id is the
same as service id and implcitly the response must then be the following integer,
e.g. 6 (request) and 7 (response).

00. `/obs/state`: obs -> corin
01. `/ego/state`: ego -> corin
02. `/onc/state`: onc -> corin
03. `/corin/obs/target`: corin -> obs
04. `/corin/ego/target`: corin -> ego
05. `/corin/onc/target`: corin -> onc
06. `/obs/enable_vehicle`: obs <- corin
07. `/obs/enable_vehicle`: obs -> corin
08. `/ego/enable_vehicle`: ego <- corin
09. `/ego/enable_vehicle`: ego -> corin
10. `/onc/enable_vehicle`: onc <- corin
11. `/onc/enable_vehicle`: onc -> corin

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

- [00,01,02] pub `/abconnect/corin/state`
- [03,04,05] sub `/abconnect/corin/target`
- [06,08,10] sub `/abconnect/corin/srv/req/enable_vehicle`
- [07,09,11] pub `/abconnect/corin/srv/res/enable_vehicle`

90. `/rsu/objectposes`: rsu -> corin

### Docker

1. Build with `util/build`
2. Enter the docker container with `util/run`
3. Run example application inside the container with `roslaunch city_lmpc sml.launch`
