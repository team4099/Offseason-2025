import {NT4_Client} from "../Lib/NT4.js";

const branchButtons = document.querySelectorAll(".branch");
const algaeButtons = document.querySelectorAll(".algae");
let coralStates = Array.from(document.querySelectorAll(".branch")).map(coral => coral.classList.contains("active"));
branchButtons.forEach(btn => {
    btn.addEventListener("click", (event) => {
        console.log(branchButtons);
        btn.classList.toggle("active");
        coralStates = Array.from(document.querySelectorAll(".branch")).map(coral => coral.classList.contains("active"));
        console.log(coralStates);
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
const prioritiesTopicName = "Priorities"

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
        } else if (topic.name === toDashboardPrefix + prioritiesTopicName) {
            priorities = value;
        }
        else {
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
    ntClient.publishTopic(toRobotPrefix + prioritiesTopicName, "string[]");
    ntClient.connect();

    initializeDraggableList();
});



let algaeStates = Array.from(document.querySelectorAll(".algae")).map(algae => algae.classList.contains("active"));
let priorities = getListOrder();

ntClient.addSample(toRobotPrefix + coralTopicName, coralStates);
ntClient.addSample(toRobotPrefix + algaeTopicName, algaeStates);

function initializeDraggableList() {
    const listContainer = document.getElementById('listContainer');
    const listItems = document.querySelectorAll('.list-item');

    let draggedElement = null;

    listItems.forEach(item => {
        item.addEventListener('dragstart', (e) => {
            draggedElement = item;
            item.classList.add('dragging');
            e.dataTransfer.effectAllowed = 'move';
        });

        item.addEventListener('dragend', (e) => {
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

        item.addEventListener('dragleave', (e) => {
            item.classList.remove('drag-over');
        });

        item.addEventListener('drop', (e) => {
            e.preventDefault();
            item.classList.remove('drag-over');

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
        });
    });
}

function getListOrder() {
     const listContainer = document.getElementById('listContainer');
    const listItems = listContainer.querySelectorAll('.list-item');
    return Array.from(listItems).map(item => item.textContent.trim());

}


