const { Service, Param } = require("roslib");
const { ABConnect, ROSPlugin } = require("abconnect-sdk-lite");

const ROS_PLUGIN = new ROSPlugin(9090, true);

let ros = ROS_PLUGIN.ros;

let hostname = require("os").hostname();

let n = process.argv[2];
let master = process.argv[3];
let vehicles = process.argv.slice(4, n);

let ids = {
    // Transitions
    "debby-(transition)>svea2": 00,
    "debby-(transition)>svea5": 01,
    // Messages from SVEA2
    "svea2-(outgoing)>debby": 02,
    "svea2-(outgoing)>svea5": 04,
    // Messages from SVEA5
    "svea5-(outgoing)>debby": 05,
    "svea5-(outgoing)>svea2": 06,
};

function main() {

    const abconnect = new ABConnect(
        {host: "wss://intelligent-cluster-manager.herokuapp.com/signal"},
        {id: hostname},
        ROS_PLUGIN
    );

    if (hostname === master) {

        // master -> vehicle
        vehicles.forEach(name => {
            abconnect.addDeviceById(name).then(client => {
                client.addPublisher(ids[`${hostname}-(transition)>${name}`], "transition", "std_msgs/String");
                client.addSubscriber(ids[`${name}-(incoming)>${hostname}`], "incoming", "wp3_tests/Packet");
            });
        });

    } else {

        // vehicle -> master
        abconnect.addDeviceById(master).then(client => {
            client.addPublisher(ids[`${hostname}-(outgoing)>${master}`], "outgoing", "wp3_tests/Packet");
            client.addSubscriber(ids[`${master}-(transition)>${hostname}`], "transition", "std_msgs/String");
        });

        // vehicle -> vehicle
        vehicles.forEach(name => {
            abconnect.addDeviceById(name).then(client => {
                client.addPublisher(ids[`${hostname}-(outgoing)>${name}`], "outgoing", "wp3_tests/Packet");
                client.addSubscriber(ids[`${name}-(incoming)>${hostname}`], "incoming", "wp3_tests/Packet");
            });
        });

    }

}

main();
