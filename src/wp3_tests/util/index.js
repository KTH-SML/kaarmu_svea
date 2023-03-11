const { Service, Param } = require("roslib");
const { ABConnect, ROSPlugin } = require("abconnect-sdk-lite");

const ROS_PLUGIN = new ROSPlugin(9090, true);

let ros = ROS_PLUGIN.ros;

let hostname = require("os").hostname();

let n = process.argv[2];
let master = process.argv[3];
let vehicles = process.argv.slice(4, n);

function main() {

    const abconnect = new ABConnect(
        {host: "wss://intelligent-cluster-manager.herokuapp.com/signal"},
        {id: hostname},
        ROS_PLUGIN
    );

    let ids = {};
    let counter = 0;
    let register = key => {
        if (ids[key] === undefined)
            ids[key] = counter++;
    };

    // For master...
    vehicles.forEach(name => {
        register(`${master}-(transition)>${name}`); // ... `transition` topics
        register(`${master}-(incoming)>${name}`);   // ... `incoming` topics
    });

    // For each vehicle...
    vehicles.forEach(self => {
        register(`${self}-(outgoing)>${master}`);   // ... `outgoing` to master
        register(`${master}-(transition)>${self}`); // ... `transition` from master
        vehicles.forEach(name => {
            if (name === self) return; // skip self
            register(`${self}-(outgoing)>${name}`); // ... `outgoing` from self
            register(`${name}-(incoming)>${self}`); // ... `incoming` to self
        });
    });

    // Create the connections
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
