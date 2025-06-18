// server.js
// A Node.js Express server to calculate left-turn-only routes.

const express = require('express');
const cors = require('cors');
const axios = require('axios');

// --- SERVER SETUP ---
const app = express();
// IMPORTANT: Use the port provided by the hosting environment, or 3000 for local testing.
const PORT = process.env.PORT || 3000;

app.use(cors());
app.use(express.json());

// --- CONSTANTS AND CONFIGURATION ---
const OVERPASS_API_URL = 'https://overpass-api.de/api/interpreter';
// Define highway types suitable for driving
const HIGHWAY_FILTER = `[highway~"^(motorway|trunk|primary|secondary|tertiary|unclassified|residential|motorway_link|trunk_link|primary_link|secondary_link|tertiary_link|living_street|service)$"]`;
// Turn angle definitions (in degrees)
const LEFT_TURN_ANGLE_MIN = -150; // -30 to -150 degrees is a left turn
const LEFT_TURN_ANGLE_MAX = -30;
const STRAIGHT_ANGLE_THRESHOLD = 30; // Anything within +/- 30 degrees is "straight"

// --- PRIORITY QUEUE IMPLEMENTATION ---
// A simple min-priority queue for the A* algorithm's open set.
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
            return res.status(404).json({ message: 'Could not fetch map data for the specified area. Try a different location.' });
        }

        console.log('2. Building graph...');
        const { graph, nodes } = buildGraph(osmData);

        console.log('3. Finding closest nodes to start/end points...');
        const startNodeId = findClosestNode(start, nodes);
        const endNodeId = findClosestNode(end, nodes);
        if (!startNodeId || !endNodeId) {
            return res.status(404).json({ message: 'Could not map start/end points to the road network.' });
        }
        
        console.log(`4. Running A* from ${startNodeId} to ${endNodeId}...`);
        const resultPath = findPath(startNodeId, endNodeId, graph, nodes);

        if (!resultPath) {
            return res.status(404).json({ message: 'No valid left-turn-only path could be found. This can happen in areas with many one-way streets or where no such path exists.' });
        }
        
        console.log('5. Path found! Reconstructing and sending response.');
        // Reconstruct path with full coordinate objects
        const detailedPath = resultPath.path.map(id => ({ lat: nodes[id].lat, lon: nodes[id].lon }));
        
        // Add start and end coordinates to the path extremities for precise visualization
        const finalPath = [
            { lat: start.lat, lng: start.lng },
            ...detailedPath.map(p => ({ lat: p.lat, lng: p.lon })),
            { lat: end.lat, lng: end.lng }
        ];

        res.json({ path: finalPath, distance: resultPath.distance });

    } catch (error) {
        console.error('Error on /route:', error.message);
        res.status(500).json({ message: 'An internal server error occurred.' });
    }
});


// --- CORE LOGIC ---

/**
 * Fetches OSM road network data for a given bounding box.
 * @param {object} start - {lat, lng} for start point.
 * @param {object} end - {lat, lng} for end point.
 * @returns {Promise<object>} The OSM data from Overpass API.
 */
async function fetchOsmData(start, end) {
    // Create a bounding box with a buffer
    const buffer = 0.05; 
    const minLat = Math.min(start.lat, end.lat) - buffer;
    const maxLat = Math.max(start.lat, end.lat) + buffer;
    const minLng = Math.min(start.lng, end.lng) - buffer;
    const maxLng = Math.max(start.lng, end.lng) + buffer;
    const bbox = `${minLat},${minLng},${maxLat},${maxLng}`;
    
    // Overpass QL query
    const query = `
        [out:json][timeout:25];
        (
          way${HIGHWAY_FILTER}(bbox:${bbox});
        );
        (._;>;);
        out;`;

    const response = await axios.post(OVERPASS_API_URL, `data=${encodeURIComponent(query)}`);
    return response.data;
}

/**
 * Builds an adjacency list graph from OSM data.
 * @param {object} osmData - The raw data from the Overpass API.
 * @returns {{graph: object, nodes: object}} - The graph and a map of node details.
 */
function buildGraph(osmData) {
    const nodes = {};
    const graph = {};

    // First pass: collect all node coordinates
    osmData.elements.forEach(el => {
        if (el.type === 'node') {
            nodes[el.id] = { lat: el.lat, lon: el.lon };
            graph[el.id] = [];
        }
    });

    // Second pass: build graph edges from ways
    osmData.elements.forEach(el => {
        if (el.type === 'way' && el.nodes) {
            for (let i = 0; i < el.nodes.length - 1; i++) {
                const nodeA = el.nodes[i];
                const nodeB = el.nodes[i+1];
                
                // Add edge from A to B
                if (graph[nodeA] && nodes[nodeB]) { // Ensure both nodes exist
                    graph[nodeA].push(nodeB);
                }

                // If not a one-way street, add edge from B to A
                if (el.tags?.oneway !== 'yes') {
                     if (graph[nodeB] && nodes[nodeA]) { // Ensure both nodes exist
                        graph[nodeB].push(nodeA);
                    }
                }
            }
        }
    });

    return { graph, nodes };
}

/**
 * Finds the closest node in the graph to a given coordinate.
 * @param {{lat: number, lng: number}} point - The coordinate to match.
 * @param {object} nodes - The map of all nodes from the graph.
 * @returns {string|null} The ID of the closest node.
 */
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
 * @param {string} startId - The ID of the starting node.
 * @param {string} endId - The ID of the destination node.
 * @param {object} graph - The adjacency list of the graph.
 * @param {object} nodes - The map of all nodes with coordinates.
 * @returns {object|null} - An object with the path and total distance, or null if no path is found.
 */
function findPath(startId, endId, graph, nodes) {
    const openSet = new PriorityQueue();
    // The "cameFrom" map stores the path. Key is a node ID, value is the node ID it came from.
    // We need a unique key for each state (current, previous), so we'll use "currentNodeId:previousNodeId"
    const cameFrom = {};
    // gScore stores the cost (distance) to get to a node.
    const gScore = {}; // Key: "currentNodeId:previousNodeId"

    // Initialize scores for the starting node
    const startKey = `${startId}:null`; // No previous node at the start
    gScore[startKey] = 0;
    const startHeuristic = haversineDistance(nodes[startId], nodes[endId]);
    openSet.enqueue({ current: startId, prev: null }, startHeuristic);

    while (!openSet.isEmpty()) {
        const { current, prev } = openSet.dequeue();

        if (current === endId) {
            // Path found! Reconstruct it.
            return reconstructPath(cameFrom, current, prev, nodes);
        }

        const neighbors = graph[current] || [];
        const availableTurns = [];

        // First pass: identify all possible turns (left, straight) from the current node
        for (const neighbor of neighbors) {
            if (neighbor === prev) continue; // Avoid going back immediately

            // For the very first step, any direction is fine
            if (prev === null) {
                availableTurns.push({ type: 'straight', neighbor }); // Treat first move as "straight"
                continue;
            }

            const angle = calculateTurnAngle(nodes[prev], nodes[current], nodes[neighbor]);
            
            if (angle >= LEFT_TURN_ANGLE_MIN && angle <= LEFT_TURN_ANGLE_MAX) {
                availableTurns.push({ type: 'left', neighbor });
            } else if (Math.abs(angle) < STRAIGHT_ANGLE_THRESHOLD) {
                availableTurns.push({ type: 'straight', neighbor });
            }
        }
        
        // Prioritize left turns. If any exist, only consider those.
        const leftTurns = availableTurns.filter(t => t.type === 'left');
        const straightTurns = availableTurns.filter(t => t.type === 'straight');
        
        const turnsToProcess = leftTurns.length > 0 ? leftTurns : straightTurns;

        // Second pass: process the chosen turns
        for (const turn of turnsToProcess) {
            const { neighbor } = turn;
            const currentKey = `${current}:${prev}`;
            const distanceToNeighbor = haversineDistance(nodes[current], nodes[neighbor]);
            
            // Add a slight penalty for going straight to prioritize left turns of similar length
            const turnPenalty = turn.type === 'straight' ? 1.05 : 1;
            
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

/**
 * Calculates Haversine distance between two lat/lon points.
 * @param {{lat: number, lon: number}} p1 - Point 1
 * @param {{lat: number, lon: number}} p2 - Point 2
 * @returns {number} Distance in meters.
 */
function haversineDistance(p1, p2) {
    const R = 6371e3; // meters
    const φ1 = p1.lat * Math.PI/180;
    const φ2 = p2.lat * Math.PI/180;
    const Δφ = (p2.lat-p1.lat) * Math.PI/180;
    const Δλ = (p2.lon-p1.lon) * Math.PI/180;

    const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
              Math.cos(φ1) * Math.cos(φ2) *
              Math.sin(Δλ/2) * Math.sin(Δλ/2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

    return R * c;
}

/**
 * Calculates the angle of a turn.
 * @param {{lat, lon}} p1 - Previous point.
 * @param {{lat, lon}} p2 - Current point.
 * @param {{lat, lon}} p3 - Next point.
 * @returns {number} The angle in degrees. Negative for left, positive for right.
 */
function calculateTurnAngle(p1, p2, p3) {
    const bearing1 = Math.atan2(p2.lon - p1.lon, p2.lat - p1.lat);
    const bearing2 = Math.atan2(p3.lon - p2.lon, p3.lat - p2.lat);
    
    let angle = (bearing2 - bearing1) * (180 / Math.PI);
    
    if (angle > 180) angle -= 360;
    if (angle < -180) angle += 360;
    
    return angle;
}

/**
 * Reconstructs the path from the 'cameFrom' map after A* completes.
 * @returns {{path: Array<string>, distance: number}}
 */
function reconstructPath(cameFrom, currentId, prevId, nodes) {
    const path = [currentId];
    let totalDistance = 0;
    
    let currentKey = `${currentId}:${prevId}`;
    let previousNode = nodes[prevId];

    while (cameFrom[currentKey]) {
        const { current, prev } = cameFrom[currentKey];
        
        totalDistance += haversineDistance(nodes[path[0]], previousNode); // Add distance of last segment
        
        path.unshift(current);
        previousNode = nodes[prev];
        currentKey = `${current}:${prev}`;
    }
    
    return { path: path.map(String), distance: totalDistance };
}


// --- START SERVER ---
app.listen(PORT, () => {
    console.log(`Left-Turn Router server is running on http://localhost:${PORT}`);
});
