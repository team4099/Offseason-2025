import {NT4_Client} from "../Lib/NT4.js";

// Topic paths
const toRobotPrefix = "/ReefControls/ToRobot/";
const toDashboardPrefix = "/ReefControls/ToDashboard/";
const coralTopicName = "Coral";
const algaeTopicName = "Algae";
const prioritiesTopicName = "Priorities";

// State variables
let coralStates = [];
let algaeStates = [];
let priorities = [];
let ntClient = null;
let isConnected = false;

// DOM elements (will be initialized after page load)
let branchButtons = null;
let algaeButtons = null;

// Initialize button event listeners
function initializeButtons() {
    branchButtons = document.querySelectorAll(".branch");
    algaeButtons = document.querySelectorAll(".algae");

    branchButtons.forEach(btn => {
        btn.addEventListener("click", () => {
            btn.classList.toggle("active");
            coralStates = Array.from(branchButtons).map(coral => coral.classList.contains("active"));

            if (isConnected && ntClient) {
                ntClient.addSample(toRobotPrefix + coralTopicName, coralStates);
                console.log("Coral states:", coralStates);
            }
        });
    });

    algaeButtons.forEach(btn => {
        btn.addEventListener("click", () => {
            btn.classList.toggle("active");
            algaeStates = Array.from(algaeButtons).map(algae => algae.classList.contains("active"));

            if (isConnected && ntClient) {
                ntClient.addSample(toRobotPrefix + algaeTopicName, algaeStates);
                console.log("Algae states:", algaeStates);
            }
        });
    });
}

// Initialize draggable priority list
function initializeDraggableList() {
    const listContainer = document.getElementById('listContainer');
    if (!listContainer) {
        console.warn("List container not found");
        return;
    }

    const listItems = document.querySelectorAll('.list-item');

    let draggedElement = null;

    listItems.forEach(item => {
        item.addEventListener('dragstart', (e) => {
            draggedElement = item;
            item.classList.add('dragging');
            e.dataTransfer.effectAllowed = 'move';
        });

        item.addEventListener('dragend', () => {
            item.classList.remove('dragging');
            document.querySelectorAll('.list-item').forEach(el => {
                el.classList.remove('drag-over');
            });
        });

        item.addEventListener('dragover', (e) => {
            e.preventDefault();
            e.dataTransfer.dropEffect = 'move';

            if (item !== draggedElement) {
                item.classList.add('drag-over');
            }
        });

        item.addEventListener('dragleave', () => {
            item.classList.remove('drag-over');
        });

        item.addEventListener('drop', (e) => {
            e.preventDefault();

            if (item !== draggedElement) {
                const allItems = [...listContainer.querySelectorAll('.list-item')];
                const draggedIndex = allItems.indexOf(draggedElement);
                const targetIndex = allItems.indexOf(item);

                if (draggedIndex < targetIndex) {
                    item.parentNode.insertBefore(draggedElement, item.nextSibling);
                } else {
                    item.parentNode.insertBefore(draggedElement, item);
                }
            }

            priorities = getListOrder();
            if (isConnected && ntClient) {
                ntClient.addSample(toRobotPrefix + prioritiesTopicName, priorities);
                console.log("Priorities updated:", priorities);
            }
        });
    });
}

// Get current list order
function getListOrder() {
    const listContainer = document.getElementById('listContainer');
    if (!listContainer) return [];

    const listItems = listContainer.querySelectorAll('.list-item');
    return Array.from(listItems).map(item => item.textContent.trim());
}

// Send initial states after connection
function sendInitialStates() {
    if (!isConnected || !ntClient) {
        console.warn("Not connected, skipping initial states");
        return;
    }

    coralStates = Array.from(branchButtons).map(coral => coral.classList.contains("active"));
    algaeStates = Array.from(algaeButtons).map(algae => algae.classList.contains("active"));
    priorities = getListOrder();

    console.log("Sending initial states:", {coralStates, algaeStates, priorities});

    // Small delay to ensure topics are fully published
    setTimeout(() => {
        if (isConnected && ntClient) {
            ntClient.addSample(toRobotPrefix + coralTopicName, coralStates);
            ntClient.addSample(toRobotPrefix + algaeTopicName, algaeStates);
            ntClient.addSample(toRobotPrefix + prioritiesTopicName, priorities);
        }
    }, 100);
}

// Update UI based on received data
function updateCoralUI() {
    if (!branchButtons) return;

    branchButtons.forEach((btn, index) => {
        if (coralStates[index]) {
            btn.classList.add("active");
        } else {
            btn.classList.remove("active");
        }
    });
}

function updateAlgaeUI() {
    if (!algaeButtons) return;

    algaeButtons.forEach((btn, index) => {
        if (algaeStates[index]) {
            btn.classList.add("active");
        } else {
            btn.classList.remove("active");
        }
    });
}

function updatePrioritiesUI() {
    console.log("Priorities received from robot:", priorities);
}

// Initialize on page load
window.addEventListener("load", () => {
    console.log("Page loaded, initializing...");

    // Initialize UI first
    initializeButtons();
    initializeDraggableList();

    // Initialize NT4 Client
    ntClient = new NT4_Client(
        window.location.hostname,
        "ReefControls",
        (topic) => {
            // Topic announce
            console.log("Topic announced:", topic.name);
        },
        (topic) => {
            // Topic unannounce
            console.log("Topic unannounced:", topic.name);
        },
        (topic, timestamp, value) => {
            // New data received from robot
            console.log("Data received:", topic.name, value);

            if (topic.name === toDashboardPrefix + coralTopicName) {
                coralStates = value;
                updateCoralUI();
            } else if (topic.name === toDashboardPrefix + algaeTopicName) {
                algaeStates = value;
                updateAlgaeUI();
            } else if (topic.name === toDashboardPrefix + prioritiesTopicName) {
                priorities = value;
                updatePrioritiesUI();
            }
        },
        () => {
            // Connected
            console.log("Connected to NT4 server");
            isConnected = true;
            document.body.style.backgroundColor = "";

            // Send initial states after a short delay
            sendInitialStates();
        },
        () => {
            // Disconnected
            console.log("Disconnected from NT4 server");
            isConnected = false;
            document.body.style.backgroundColor = "red";
        }
    );

    // Subscribe to topics
    ntClient.subscribe(
        [
            toDashboardPrefix + coralTopicName,
            toDashboardPrefix + algaeTopicName,
            toDashboardPrefix + prioritiesTopicName,
        ],
        false,
        false,
        0.02
    );

    // Publish topics
    ntClient.publishTopic(toRobotPrefix + coralTopicName, "boolean[]");
    ntClient.publishTopic(toRobotPrefix + algaeTopicName, "boolean[]");
    ntClient.publishTopic(toRobotPrefix + prioritiesTopicName, "string[]");

    // Connect to NT4 server
    ntClient.connect();
});