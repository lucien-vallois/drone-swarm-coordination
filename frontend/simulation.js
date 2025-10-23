// Global variables
let scene, camera, renderer, controls;
let drones = new Map();
let obstacles = [];
let targetPositions = new Map();
let websocket = null;
let animationId = null;
let currentState = null;
let droneLabels = new Map();

// Colors
const DRONE_COLOR = 0x00ff00;
const TARGET_COLOR = 0xff0000;
const OBSTACLE_COLOR = 0xffa500;
const PATH_COLOR = 0x0000ff;

// Initialize Three.js scene
function init() {
    // Create scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x87CEEB); // Sky blue

    // Create camera
    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(60, 60, 60);
    camera.lookAt(50, 25, 25);

    // Create renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    document.getElementById('container').appendChild(renderer.domElement);

    // Create controls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;

    // Add lighting
    addLighting();

    // Add ground
    addGround();

    // Add coordinate system
    addCoordinateSystem();

    // Connect to WebSocket
    connectWebSocket();

    // Start animation loop
    animate();
}

function addLighting() {
    // Ambient light
    const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
    scene.add(ambientLight);

    // Directional light (sun)
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(50, 50, 50);
    directionalLight.castShadow = true;
    directionalLight.shadow.mapSize.width = 2048;
    directionalLight.shadow.mapSize.height = 2048;
    directionalLight.shadow.camera.near = 0.5;
    directionalLight.shadow.camera.far = 500;
    directionalLight.shadow.camera.left = -100;
    directionalLight.shadow.camera.right = 100;
    directionalLight.shadow.camera.top = 100;
    directionalLight.shadow.camera.bottom = -100;
    scene.add(directionalLight);
}

function addGround() {
    // Ground plane
    const groundGeometry = new THREE.PlaneGeometry(150, 150);
    const groundMaterial = new THREE.MeshLambertMaterial({
        color: 0x90EE90,
        transparent: true,
        opacity: 0.8
    });
    const ground = new THREE.Mesh(groundGeometry, groundMaterial);
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = 0;
    ground.receiveShadow = true;
    scene.add(ground);

    // Grid helper
    const gridHelper = new THREE.GridHelper(100, 10, 0x000000, 0x444444);
    gridHelper.position.y = 0.01;
    scene.add(gridHelper);
}

function addCoordinateSystem() {
    // X axis (red)
    const xAxis = new THREE.ArrowHelper(
        new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(0, 0, 0),
        10, 0xff0000
    );
    scene.add(xAxis);

    // Y axis (green)
    const yAxis = new THREE.ArrowHelper(
        new THREE.Vector3(0, 1, 0),
        new THREE.Vector3(0, 0, 0),
        10, 0x00ff00
    );
    scene.add(yAxis);

    // Z axis (blue)
    const zAxis = new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, 1),
        new THREE.Vector3(0, 0, 0),
        10, 0x0000ff
    );
    scene.add(zAxis);
}

function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.host || 'localhost:8765';
    const wsUrl = `${protocol}//${host}`;

    websocket = new WebSocket(wsUrl);

    websocket.onopen = function(event) {
        updateStatus('connection-status', 'Connected', 'green');
        updateStatus('simulation-status', 'Ready', 'blue');

        // Request initial state
        sendMessage({
            type: 'request',
            request: 'state'
        });

        sendMessage({
            type: 'request',
            request: 'config'
        });
    };

    websocket.onmessage = function(event) {
        try {
            const data = JSON.parse(event.data);
            handleMessage(data);
        } catch (error) {
            console.error('Failed to parse WebSocket message:', error);
        }
    };

    websocket.onclose = function(event) {
        updateStatus('connection-status', 'Disconnected', 'red');
        updateStatus('simulation-status', 'Offline', 'red');

        // Attempt to reconnect after 5 seconds
        setTimeout(connectWebSocket, 5000);
    };

    websocket.onerror = function(error) {
        console.error('WebSocket error:', error);
        updateStatus('connection-status', 'Error', 'red');
    };
}

function handleMessage(data) {
    if (data.type === 'state' && data.data) {
        currentState = data.data;
        updateVisualization();
        updateStats();
    } else if (data.type === 'config' && data.data) {
        updateConfig(data.data);
    } else if (data.status) {
        updateStatus('simulation-status', data.status, 'green');
    } else if (data.error) {
        console.error('Server error:', data.error);
        updateStatus('simulation-status', 'Error: ' + data.error, 'red');
    }
}

function updateVisualization() {
    if (!currentState) return;

    // Update drones
    updateDrones();

    // Update obstacles
    updateObstacles();

    // Update target positions
    updateTargets();
}

function updateDrones() {
    const coordinatorDrones = currentState.coordinator.drones || {};

    // Remove drones that no longer exist
    for (const [droneId, droneMesh] of drones) {
        if (!(droneId in coordinatorDrones)) {
            scene.remove(droneMesh);
            drones.delete(droneId);

            // Remove label
            const label = droneLabels.get(droneId);
            if (label) {
                document.body.removeChild(label);
                droneLabels.delete(droneId);
            }
        }
    }

    // Update or create drones
    for (const [droneId, droneData] of Object.entries(coordinatorDrones)) {
        const id = parseInt(droneId);
        const position = droneData.position;
        const isActive = droneData.is_active;

        let droneMesh = drones.get(id);

        if (!droneMesh) {
            // Create new drone
            const geometry = new THREE.CylinderGeometry(0.3, 0.3, 0.1, 8);
            const material = new THREE.MeshPhongMaterial({
                color: isActive ? DRONE_COLOR : 0x666666,
                shininess: 100
            });
            droneMesh = new THREE.Mesh(geometry, material);
            droneMesh.castShadow = true;
            droneMesh.receiveShadow = true;

            // Add propellers
            for (let i = 0; i < 4; i++) {
                const propGeometry = new THREE.CylinderGeometry(0.8, 0.8, 0.02, 8);
                const propMaterial = new THREE.MeshPhongMaterial({ color: 0x333333 });
                const propeller = new THREE.Mesh(propGeometry, propMaterial);
                propeller.position.x = 0.5 * Math.cos(i * Math.PI / 2);
                propeller.position.z = 0.5 * Math.sin(i * Math.PI / 2);
                propeller.position.y = 0.06;
                propeller.rotation.x = Math.PI / 2;
                droneMesh.add(propeller);
            }

            scene.add(droneMesh);
            drones.set(id, droneMesh);

            // Add label
            const label = document.createElement('div');
            label.className = 'drone-label';
            label.textContent = `Drone ${id}`;
            document.body.appendChild(label);
            droneLabels.set(id, label);
        }

        // Update position
        droneMesh.position.set(position[0], position[2], position[1]); // Note: Y/Z swap for 3D view

        // Update color based on activity
        droneMesh.material.color.setHex(isActive ? DRONE_COLOR : 0x666666);

        // Update label position
        updateDroneLabel(id, position);
    }
}

function updateDroneLabel(droneId, position) {
    const label = droneLabels.get(droneId);
    if (!label) return;

    const vector = new THREE.Vector3(position[0], position[2], position[1]);
    vector.project(camera);

    const x = (vector.x * 0.5 + 0.5) * window.innerWidth;
    const y = (-vector.y * 0.5 + 0.5) * window.innerHeight;

    label.style.left = `${x}px`;
    label.style.top = `${y - 30}px`; // Offset above drone
}

function updateObstacles() {
    const environmentObstacles = currentState.environment?.obstacles || [];

    // Remove old obstacles
    obstacles.forEach(obstacle => scene.remove(obstacle));
    obstacles = [];

    // Add new obstacles
    environmentObstacles.forEach(obs => {
        const geometry = new THREE.SphereGeometry(obs.radius, 16, 16);
        const material = new THREE.MeshPhongMaterial({
            color: OBSTACLE_COLOR,
            transparent: true,
            opacity: 0.7
        });
        const obstacleMesh = new THREE.Mesh(geometry, material);
        obstacleMesh.position.set(obs.position[0], obs.position[2], obs.position[1]);
        obstacleMesh.castShadow = true;
        obstacleMesh.receiveShadow = true;

        scene.add(obstacleMesh);
        obstacles.push(obstacleMesh);
    });
}

function updateTargets() {
    const targets = currentState.coordinator?.target_positions || {};

    // Remove old targets
    for (const [droneId, targetMesh] of targetPositions) {
        scene.remove(targetMesh);
        targetPositions.delete(droneId);
    }

    // Add new targets
    for (const [droneId, targetPos] of Object.entries(targets)) {
        const id = parseInt(droneId);
        const geometry = new THREE.SphereGeometry(0.2, 8, 8);
        const material = new THREE.MeshBasicMaterial({
            color: TARGET_COLOR,
            transparent: true,
            opacity: 0.6
        });
        const targetMesh = new THREE.Mesh(geometry, material);
        targetMesh.position.set(targetPos[0], targetPos[2], targetPos[1]);

        scene.add(targetMesh);
        targetPositions.set(id, targetMesh);
    }
}

function updateStats() {
    if (!currentState) return;

    document.getElementById('drone-count').textContent = Object.keys(currentState.coordinator?.drones || {}).length;
    document.getElementById('fps').textContent = currentState.fps?.toFixed(1) || '0';
    document.getElementById('frame').textContent = currentState.frame || '0';
    document.getElementById('leader').textContent = currentState.coordinator?.leader_id ?? 'None';
}

function updateConfig(config) {
    // Update UI with config
    console.log('Config received:', config);
}

function sendMessage(message) {
    if (websocket && websocket.readyState === WebSocket.OPEN) {
        websocket.send(JSON.stringify(message));
    }
}

function updateStatus(elementId, text, color) {
    const element = document.getElementById(elementId);
    if (element) {
        element.textContent = text;
        element.style.color = color;
    }
}

function animate() {
    animationId = requestAnimationFrame(animate);

    // Update controls
    controls.update();

    // Update drone labels
    if (currentState?.coordinator?.drones) {
        for (const [droneId, droneData] of Object.entries(currentState.coordinator.drones)) {
            updateDroneLabel(parseInt(droneId), droneData.position);
        }
    }

    renderer.render(scene, camera);
}

// Event listeners
document.getElementById('start-btn').addEventListener('click', () => {
    sendMessage({
        type: 'command',
        command: 'start'
    });
});

document.getElementById('stop-btn').addEventListener('click', () => {
    sendMessage({
        type: 'command',
        command: 'stop'
    });
});

document.getElementById('reset-btn').addEventListener('click', () => {
    sendMessage({
        type: 'command',
        command: 'reset'
    });
});

document.getElementById('add-obstacle-btn').addEventListener('click', () => {
    const x = parseFloat(document.getElementById('obs-x').value) || 50;
    const y = parseFloat(document.getElementById('obs-y').value) || 50;
    const z = parseFloat(document.getElementById('obs-z').value) || 10;
    const radius = parseFloat(document.getElementById('obs-radius').value) || 3;

    sendMessage({
        type: 'command',
        command: 'add_obstacle',
        params: { x, y, z, radius }
    });
});

// Formation buttons
document.querySelectorAll('.formation-option').forEach(button => {
    button.addEventListener('click', () => {
        // Remove active class from all buttons
        document.querySelectorAll('.formation-option').forEach(btn => {
            btn.classList.remove('active');
        });

        // Add active class to clicked button
        button.classList.add('active');

        const formation = button.dataset.formation;
        sendMessage({
            type: 'command',
            command: 'set_formation',
            params: { formation }
        });
    });
});

// Window resize
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// Initialize
init();
