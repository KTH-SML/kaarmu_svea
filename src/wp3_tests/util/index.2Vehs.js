const { Service, Param } = require("roslib");
const { ABConnect, ROSPlugin } = require("abconnect-sdk-lite");

const ROS_PLUGIN = new ROSPlugin(9090, true);

let ros = ROS_PLUGIN.ros;

function debby() {

    const abconnect = new ABConnect(
        {host: "wss://intelligent-cluster-manager.herokuapp.com/signal"},
        {id: "debby"},
        ROS_PLUGIN
    );

    abconnect.addDeviceById("svea2").then(client => {
        client.addPublisher(00, "transition", "std_msgs/String");
        client.addSubscriber(02, "incoming", "wp3_tests/Packet");
    });

    abconnect.addDeviceById("svea5").then(client => {
        client.addPublisher(01, "transition", "std_msgs/String");
        client.addSubscriber(05, "incoming", "wp3_tests/Packet");
    });

}

function svea2() {

    const abconnect = new ABConnect(

        {host: "wss://intelligent-cluster-manager.herokuapp.com/signal"},
        {id: "svea2"},
        ROS_PLUGIN
    );

    abconnect.addDeviceById("debby").then(client => {
        client.addSubscriber(00, "transition", "std_msgs/String");
        client.addPublisher(02, "outgoing", "wp3_tests/Packet");
    });

    abconnect.addDeviceById("svea5").then(client => {
        client.addPublisher(04, "outgoing", "wp3_tests/Packet");
        client.addSubscriber(06, "incoming", "wp3_tests/Packet");
    });

}

function svea5() {

    const abconnect = new ABConnect(
        {host: "wss://intelligent-cluster-manager.herokuapp.com/signal"},
        {id: "svea5"},
        ROS_PLUGIN
    );

    abconnect.addDeviceById("debby").then(client => {
        client.addSubscriber(01, "transition", "std_msgs/String");
        client.addPublisher(05, "outgoing", "wp3_tests/Packet");
    });

    abconnect.addDeviceById("svea2").then(client => {
        client.addPublisher(06, "outgoing", "wp3_tests/Packet");
        client.addSubscriber(04, "incoming", "wp3_tests/Packet");
    });

}

let hostname = require("os").hostname();

switch (hostname) {
    case "debby":
        debby();
        break;
    case "svea2":
        svea2();
        break;
    case "svea5":
        svea5();
        break;
}
