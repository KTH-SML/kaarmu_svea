const { Service, Param } = require("roslib");
const { ABConnect, ROSPlugin } = require("abconnect-sdk-lite");

const ROS_PLUGIN = new ROSPlugin(9090, true);

let ros = ROS_PLUGIN.ros;

let hostname = require("os").hostname();

let n = Number(process.argv[2]);
let master = process.argv[3];
let vehicles = process.argv.slice(4, 4+n);

let ids = {};
let counter = 0;
let register = key => {
    if (ids[key] === undefined)
        ids[key] = counter++;
};

// Register connections
vehicles.forEach(self => {
    register(`${master}->${self}`); // from master to self (`transition` topic)
    register(`${self}->${master}`); // from self to master
    vehicles.forEach(other => {
        if (self === other) return;
        register(`${self}->${other}`); // from self to other (`outgoing` topic)
        register(`${other}->${self}`); // from other to self (`incoming` topic)
    });
});

function main() {

    const abconnect = new ABConnect(
        {host: "wss://intelligent-cluster-manager.herokuapp.com/signal"},
        {id: hostname},
        ROS_PLUGIN
    );

    // Create the connections
    if (hostname === master) {

        // master -> vehicle
        vehicles.forEach(name => {
            abconnect.addDeviceById(name).then(client => {
                client.addPublisher(ids[`${hostname}->${name}`], "transition", "std_msgs/String");
                client.addSubscriber(ids[`${name}->${hostname}`], "incoming", "wp3_tests/Packet");
            });
        });

    } else {

        // vehicle -> master
        abconnect.addDeviceById(master).then(client => {
            client.addPublisher(ids[`${hostname}->${master}`], "outgoing", "wp3_tests/Packet");
            client.addSubscriber(ids[`${master}->${hostname}`], "transition", "std_msgs/String");
        });

        // vehicle -> vehicle
        vehicles.forEach(name => {
            abconnect.addDeviceById(name).then(client => {
                client.addPublisher(ids[`${hostname}->${name}`], "outgoing", "wp3_tests/Packet");
                client.addSubscriber(ids[`${name}->${hostname}`], "incoming", "wp3_tests/Packet");
            });
        });

    }

}

main();
