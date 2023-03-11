// const { Service, Param } = require("roslib");
// const { ABConnect, ROSPlugin } = require("abconnect-sdk-lite");
//
// const ROS_PLUGIN = new ROSPlugin(9090, true);
//
// let ros = ROS_PLUGIN.ros;

let hostname = require("os").hostname();

let n = Number(process.argv[2]);
let master = process.argv[3];
let vehicles = process.argv.slice(4, 4+n);

let table = {};
let counter = 0;
let register = key => {
    if (table[key] === undefined)
        table[key] = counter++;
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

console.log("Master:", master);
console.log("Vehicles:", vehicles);
console.log("Connection Table:", table);

function main() {

    const self = hostname;
    const abconnect = new ABConnect(
        {host: "wss://intelligent-cluster-manager.herokuapp.com/signal"},
        {id: hostname},
        ROS_PLUGIN
    );

    // Create the connections
    if (self === master) {

        vehicles.forEach(name => {
            abconnect.addDeviceById(name).then(client => {
                client.addPublisher(table[`${self}->${name}`], "transition", "std_msgs/String");
                client.addSubscriber(table[`${name}->${self}`], "incoming", "wp3_tests/Packet");
            });
        });

    } else {

        abconnect.addDeviceById(master).then(client => {
            client.addPublisher(table[`${self}->${master}`], "outgoing", "wp3_tests/Packet");
            client.addSubscriber(table[`${master}->${self}`], "transition", "std_msgs/String");
        });

        vehicles.forEach(name => {
            abconnect.addDeviceById(name).then(client => {
                client.addPublisher(table[`${self}->${name}`], "outgoing", "wp3_tests/Packet");
                client.addSubscriber(table[`${name}->${self}`], "incoming", "wp3_tests/Packet");
            });
        });

    }

}

main();
