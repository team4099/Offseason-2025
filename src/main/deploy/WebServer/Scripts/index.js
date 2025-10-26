import {NT4_Client} from "../Lib/NT4.js";

const branchButtons = document.querySelectorAll(".branch");
const algaeButtons = document.querySelectorAll(".algae");
console.log(branchButtons);
branchButtons.forEach(btn => {
    btn.addEventListener("click", (event) => {
        btn.classList.toggle("active");
    });
});
    algaeButtons.forEach(btn => {
        btn.addEventListener("click", (event) => {
            btn.classList.toggle("active");
        });
});

const toRobotPrefix = "/ReefControls/ToRobot/";
const toDashboardPrefix = "/ReefControls/ToDashboard/";
const coralTopicName = "Coral";
const algaeTopicName = "Algae";

const ntClient = new NT4_Client(
    window.location.hostname,
    "ReefControls",
    () => {
        // Topic announce
    },
    () => {
        // Topic unannounce
    },
    (topic, _, value) => {
        // New data
        if (topic.name === toDashboardPrefix + coralTopicName) {
            coralStates = value;
        } else if (topic.name === toDashboardPrefix + algaeTopicName) {
            algaeStates = value;
        } else {
            return;
        }
    },
    () => {
        // Connected
    },
    () => {
        // Disconnected
        document.body.style.backgroundColor = "red";
    }
);

// Start NT connection
window.addEventListener("load", () => {
    ntClient.subscribe(
        [
            toDashboardPrefix + coralTopicName,
            toDashboardPrefix + algaeTopicName,
        ],
        false,
        false,
        0.02
    );

    ntClient.publishTopic(toRobotPrefix + coralTopicName, "boolean[]");
    ntClient.publishTopic(toRobotPrefix + algaeTopicName, "int");
    ntClient.connect();
});


let coralStates = Array.from(document.querySelectorAll(".branch")).map(coral => coral.classList.contains("active"));
let algaeStates = Array.from(document.querySelectorAll(".algae")).map(algae => algae.classList.contains("active"));

ntClient.addSample(toRobotPrefix + coralTopicName, coralStates);
ntClient.addSample(toRobotPrefix + algaeTopicName, algaeStates);
