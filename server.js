// server.js
// A Node.js Express server to calculate left-turn-only routes.

const express = require('express');
const cors = require('cors');
const axios = require('axios');

// --- SERVER SETUP ---
const app = express();
// IMPORTANT: Use the port provided by the hosting environment, or 3000 for local testing.
const PORT = process.env.PORT || 3000;

// IMPORTANT: Configure CORS to allow requests from any origin.
app.use(cors());

app.use(express.json());

// --- CONSTANTS AND CONFIGURATION ---
const OVERPASS_API_URL = 'https://overpass-api.de/api/interpreter';
const HIGHWAY_FILTER = `[highway~"^(motorway|trunk|primary|secondary|tertiary|unclassified|residential|motorway_link|trunk_link|primary_link|secondary_link|tertiary_link|living_street|service)$"]`;

// Turn angle definitions (in degrees)
const LEFT_TURN_ANGLE_MIN = -150;
const LEFT_TURN_ANGLE_MAX = -30;
const RIGHT_TURN_ANGLE_MIN = 30;
const RIGHT_TURN_ANGLE_MAX = 150;
const STRAIGHT_ANGLE_THRESHOLD = 30;

// --- PRIORITY QUEUE IMPLEMENTATION ---
class PriorityQueue {
    constructor() {
        this.elements = [];
    }
    enqueue(element, priority) {
        this.elements.push({ element, priority });
        this.sort();
    }
    dequeue() {
        return this.elements.shift().element;
    }
    isEmpty() {
        return this.elements.length === 0;
    }
    sort() {
        this.elements.sort((a, b) => a.priority - b.priority);
    }
}


// --- API ENDPOINT ---
app.post('/route', async (req, res) => {
    try {
        const { start, end } = req.body;
        if (!start || !end || !start.lat || !start.lng || !end.lat || !end.lng) {
            return res.status(400).json({ message: 'Invalid start or end coordinates.' });
        }

        console.log('1. Fetching OSM data...');
        const osmData = await fetchOsmData(start, end);
        if (!osmData.elements || osmData.elements.length === 0) {
            return res.status(404).json({ message: 'Could not fetch map data for the specified area.' });
        }

        console.log('2. Building graph...');
        const { graph, nodes } = buildGraph(osmData);

        const startNodeId = findClosestNode(start, nodes);
        const endNodeId = findClosestNode(end, nodes);
        if (!startNodeId || !endNodeId) {
            return res.status(404).json({ message: 'Could not map start/end points to the road network.' });
        }
        
        // --- Multi-stage pathfinding ---
        let resultPath = null;
        let message = "";
        
        // Attempt 1: Strict left-turn only
        console.log('4a. Running A* (Attempt 1: Strict Left/Straight)...');
        resultPath = findPath(startNodeId, endNodeId, graph, nodes, {
            allowLeft: true, allowStraight: true, allowRight: false, allowUTurn: false
        });
        if(resultPath) message = "Route found using primarily left turns.";

        // Attempt 2: Allow U-Turns if first attempt fails
        if (!resultPath) {
            console.log('4b. Running A* (Attempt 2: Allowing U-Turns)...');
            resultPath = findPath(startNodeId, endNodeId, graph, nodes, {
                allowLeft: true, allowStraight: true, allowRight: false, allowUTurn: true
            });
            if(resultPath) message = "Route found with U-turns allowed.";
        }
        
        // Attempt 3: Allow Right Turns as a last resort
        if (!resultPath) {
            console.log('4c. Running A* (Attempt 3: Allowing Right Turns)...');
            resultPath = findPath(startNodeId, endNodeId, graph, nodes, {
                allowLeft: true, allowStraight: true, allowRight: true, allowUTurn: true
            });
            if(resultPath) message = "Route found with right turns as a last resort.";
        }

        if (!resultPath) {
            return res.status(404).json({ message: 'No valid path could be found, even with relaxed turn constraints.' });
        }
        
        console.log('5. Path found! Reconstructing and sending response.');
        const detailedPath = resultPath.path.map(id => ({ lat: nodes[id].lat, lon: nodes[id].lon }));
        const finalPath = [ { lat: start.lat, lng: start.lng }, ...detailedPath.map(p => ({ lat: p.lat, lng: p.lon })), { lat: end.lat, lng: end.lng } ];
        res.json({ path: finalPath, distance: resultPath.distance, message: message });

    } catch (error) {
        console.error('Error on /route:', error.stack);
        res.status(500).json({ message: 'An internal server error occurred.' });
    }
});


// --- CORE LOGIC ---

async function fetchOsmData(start, end) {
    const buffer = 0.05; 
    const bbox = `${Math.min(start.lat, end.lat)-buffer},${Math.min(start.lng, end.lng)-buffer},${Math.max(start.lat, end.lat)+buffer},${Math.max(start.lng, end.lng)+buffer}`;
    const query = `[out:json][timeout:25];(way${HIGHWAY_FILTER}(bbox:${bbox}););(._;>;);out;`;
    const response = await axios.post(OVERPASS_API_URL, `data=${encodeURIComponent(query)}`);
    return response.data;
}

function buildGraph(osmData) {
    const nodes = {};
    const graph = {};

    osmData.elements.forEach(el => {
        if (el.type === 'node') {
            nodes[el.id] = { lat: el.lat, lon: el.lon };
            graph[el.id] = [];
        }
    });

    osmData.elements.forEach(el => {
        if (el.type === 'way' && el.nodes) {
            for (let i = 0; i < el.nodes.length - 1; i++) {
                const nodeA = el.nodes[i];
                const nodeB = el.nodes[i+1];
                if (graph[nodeA] && nodes[nodeB]) graph[nodeA].push(nodeB);
                if (el.tags?.oneway !== 'yes') {
                     if (graph[nodeB] && nodes[nodeA]) graph[nodeB].push(nodeA);
                }
            }
        }
    });
    return { graph, nodes };
}

function findClosestNode(point, nodes) {
    let closestNodeId = null;
    let minDistance = Infinity;

    for (const nodeId in nodes) {
        const node = nodes[nodeId];
        const dist = haversineDistance({ lat: point.lat, lon: point.lng }, node);
        if (dist < minDistance) {
            minDistance = dist;
            closestNodeId = nodeId;
        }
    }
    return closestNodeId;
}

/**
 * The modified A* pathfinding algorithm implementation.
 * @param {object} config - Configuration object for allowed turns { allowLeft, allowStraight, allowRight, allowUTurn }
 */
function findPath(startId, endId, graph, nodes, config) {
    const openSet = new PriorityQueue();
    const cameFrom = {};
    const gScore = {};

    // Penalties for different turn types to guide the search
    const penalties = { left: 1.0, straight: 1.1, uturn: 2.5, right: 5.0 };

    const startKey = `${startId}:null`;
    gScore[startKey] = 0;
    openSet.enqueue({ current: startId, prev: null }, haversineDistance(nodes[startId], nodes[endId]));

    while (!openSet.isEmpty()) {
        const { current, prev } = openSet.dequeue();
        const currentKey = `${current}:${prev}`;

        if (current === endId) {
            const path = reconstructPath(cameFrom, current, prev);
            return { path, distance: gScore[currentKey] || 0 };
        }
        
        const neighbors = graph[current] || [];
        for (const neighbor of neighbors) {
            let turnPenalty = Infinity;

            if (neighbor === prev) { // This is a U-turn
                if (config.allowUTurn) {
                    turnPenalty = penalties.uturn;
                } else {
                    continue; // Skip if U-turns not allowed
                }
            } else if (prev === null) { // This is the first move
                turnPenalty = penalties.straight;
            } else {
                const angle = calculateTurnAngle(nodes[prev], nodes[current], nodes[neighbor]);
                if (config.allowLeft && angle >= LEFT_TURN_ANGLE_MIN && angle <= LEFT_TURN_ANGLE_MAX) {
                    turnPenalty = penalties.left;
                } else if (config.allowStraight && Math.abs(angle) < STRAIGHT_ANGLE_THRESHOLD) {
                    turnPenalty = penalties.straight;
                } else if (config.allowRight && angle >= RIGHT_TURN_ANGLE_MIN && angle <= RIGHT_TURN_ANGLE_MAX) {
                    turnPenalty = penalties.right;
                } else {
                    continue; // Skip disallowed turn types
                }
            }

            const distanceToNeighbor = haversineDistance(nodes[current], nodes[neighbor]);
            const tentativeGScore = gScore[currentKey] + (distanceToNeighbor * turnPenalty);
            const neighborKey = `${neighbor}:${current}`;

            if (!gScore[neighborKey] || tentativeGScore < gScore[neighborKey]) {
                cameFrom[neighborKey] = { current, prev };
                gScore[neighborKey] = tentativeGScore;
                const fScore = tentativeGScore + haversineDistance(nodes[neighbor], nodes[endId]);
                openSet.enqueue({ current: neighbor, prev: current }, fScore);
            }
        }
    }

    return null; // No path found
}


// --- HELPER FUNCTIONS ---

function haversineDistance(p1, p2) {
    const R = 6371e3;
    const φ1 = p1.lat * Math.PI/180, φ2 = p2.lat * Math.PI/180;
    const Δφ = (p2.lat-p1.lat) * Math.PI/180;
    const Δλ = (p2.lon-p1.lon) * Math.PI/180;
    const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ/2) * Math.sin(Δλ/2);
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
}

function calculateTurnAngle(p1, p2, p3) {
    const bearing1 = Math.atan2(p2.lon - p1.lon, p2.lat - p1.lat);
    const bearing2 = Math.atan2(p3.lon - p2.lon, p3.lat - p2.lat);
    let angle = (bearing2 - bearing1) * (180 / Math.PI);
    if (angle > 180) angle -= 360;
    if (angle < -180) angle += 360;
    return angle;
}

function reconstructPath(cameFrom, currentId, prevId) {
    const totalPath = [currentId];
    let currentKey = `${currentId}:${prevId}`;
    while (cameFrom[currentKey]) {
        const { current, prev } = cameFrom[currentKey];
        totalPath.unshift(current);
        currentKey = `${current}:${prev}`;
    }
    return totalPath.map(String);
}

// --- START SERVER ---
app.listen(PORT, () => {
    console.log(`Left-Turn Router server is running on http://localhost:${PORT}`);
});
